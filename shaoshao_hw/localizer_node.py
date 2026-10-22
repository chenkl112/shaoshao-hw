from collections import deque
from pathlib import Path
import json
import math
import time
import numpy as np
import yaml
from .config import load,RUNTIME
from .geometry import matrix,pose_msg,write_pose,transform
from .index import RetrievalIndex
from .policy import Policy,unique_best,quality_ok
from .ros_points import decode,encode

def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.duration import Duration
    from sensor_msgs.msg import PointCloud2,LaserScan
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_ros import TransformBroadcaster
    from shaoshao_hw_interfaces.srv import RegisterCloud
    from shaoshao_hw_interfaces.msg import LocalizationStatus
    class Localizer(Node):
        def __init__(self):
            super().__init__('hw_localizer')
            self.config=load(self.declare_parameter('config_path','').value or None)
            self.maps=Path(self.declare_parameter('maps_dir','').value)
            frames=yaml.safe_load((RUNTIME/'navigation_runtime/frames.yaml').read_text(encoding='utf-8'))
            self.center_offset=np.asarray(frames['center_offset'])
            self.lidar_offset=np.asarray(next(t['xyz'] for t in frames['static_transforms'] if t['child']=='navcar_lidar'))
            self.lidar_to_axis=np.asarray(self.config['yaw']['lidar_offset_xy_m'])
            self.nav=self.config['navigation'];self.p=self.nav['registration'];self.policy=Policy(self.p)
            self.index=RetrievalIndex(self.maps/'keyframes',self.maps/'site.pcd')
            self.frames=deque(maxlen=6);self.imu_pose=None;self.current_stamp=None
            self.client=self.create_client(RegisterCloud,'/hw/register');self.future=None
            self.tf=TransformBroadcaster(self);self.status_pub=self.create_publisher(String,'/hw/localization/status',10)
            self.typed_status_pub=self.create_publisher(LocalizationStatus,'/hw/localization/state',10)
            self.create_subscription(PointCloud2,'/cloud_registered',self.cloud,qos_profile_sensor_data)
            self.create_subscription(Odometry,'/navcar/state_estimation',self.odom,10)
            self.create_subscription(LaserScan,'/navcar/scan',self.scan,qos_profile_sensor_data)
            self.create_service(Trigger,'/hw/localization/retry',self.retry)
            self.create_timer(.1,self.tick)
            self.last_query=0.;self.search_started=time.monotonic();self.retry_at=0.
            self.batch=None;self.results=[];self.guesses=[];self.mode='global';self.last_message=None;self.seed=None
        def seconds(self,stamp):return stamp.sec+stamp.nanosec*1e-9
        def now(self):return self.get_clock().now().nanoseconds*1e-9
        def cloud(self,msg):
            if msg.header.frame_id!='navcar_odom':return
            stamp=self.seconds(msg.header.stamp)
            if abs(self.now()-stamp)>.5 or (self.frames and stamp<=self.frames[-1][0]):return
            try:points=decode(msg)
            except ValueError:return
            self.frames.append((stamp,points,msg.header.stamp))
        def odom(self,msg):
            if msg.header.frame_id!='navcar_odom':return
            stamp=self.seconds(msg.header.stamp)
            if abs(self.now()-stamp)>.5:return
            try:m=pose_msg(msg.pose.pose)
            except ValueError:return
            self.imu_pose=m;self.current_stamp=stamp
            center=m.copy();center[:3,3]+=m[:3,:3]@self.center_offset
            self.policy.odometry(center,stamp)
        def scan(self,msg):
            if msg.header.frame_id!='navcar_lidar':return
            stamp=self.seconds(msg.header.stamp)
            if abs(self.now()-stamp)>self.p['scan_timeout_s']:return
            points=[]
            for i,d in enumerate(msg.ranges):
                if math.isfinite(d) and msg.range_min<=d<msg.range_max:
                    a=msg.angle_min+i*msg.angle_increment
                    points.append((self.lidar_to_axis[0]+d*math.cos(a),self.lidar_to_axis[1]+d*math.sin(a)))
            self.policy.scan(stamp,points)
        def retry(self,request,response):
            self.policy.lose('正在重新搜索位置');self.retry_at=0.;self.search_started=time.monotonic()
            self.batch=None;self.guesses=[];self.results=[];self.seed=None
            if self.future is not None:self.future.cancel();self.future=None
            response.success=True;response.message='自动全局重定位已重新开始';return response
        def start_batch(self):
            if not self.frames or self.imu_pose is None or not self.client.service_is_ready():return
            newest=self.frames[-1]
            if self.now()-newest[0]>.5:return
            points=np.concatenate([p for t,p,_ in self.frames if newest[0]-t<=.7])
            if len(points)<1000:return
            # Bounded deterministic sampling; no random unbounded allocation.
            points=points[::max(1,math.ceil(len(points)/180000))]
            self.mode='local' if self.policy.initialized else ('verify' if self.seed is not None else 'global')
            if self.mode=='global':
                sensor=self.imu_pose.copy();sensor[:3,3]+=sensor[:3,:3]@self.lidar_offset
                local=transform(points,np.linalg.inv(sensor))
                self.guesses=self.index.candidates(local,sensor)
            else:self.guesses=[self.seed.copy() if self.mode=='verify' else self.policy.correction.copy()]
            self.batch=(points,newest[0],newest[2]);self.results=[]
            self.submit_next()
        def submit_next(self):
            if not self.guesses:
                if self.batch is not None:
                    best=unique_best(self.results,self.p) if self.mode=='global' else (self.results[0] if self.results else None)
                    if best is not None:
                        if self.mode=='global':self.seed=best[0].copy()
                        else:
                            accepted=self.policy.observe(best[0],best[1],self.batch[1],self.now(),self.mode=='verify')
                            if not accepted and self.mode=='local':
                                self.last_query=time.monotonic()-self.p['period_s']+.2
                            if self.mode=='verify' and self.policy.confirmations==0:self.seed=None
                            if accepted and self.policy.initialized:self.seed=None
                    else:
                        self.seed=None;self.policy.reject('全局候选不唯一或配准质量不足' if self.mode=='global' else '配准失败',self.now())
                        if self.mode=='local':self.last_query=time.monotonic()-self.p['period_s']+.2
                self.batch=None;return
            guess=self.guesses.pop(0)
            request=RegisterCloud.Request();request.source=encode(self.batch[0],'navcar_odom',self.batch[2])
            write_pose(request.initial_guess,guess)
            self.future=self.client.call_async(request);self.query_started=time.monotonic()
        def tick(self):
            now=self.now();wall=time.monotonic()
            if self.future is not None:
                if self.future.done():
                    try:
                        result=self.future.result()
                        metrics={'converged':result.converged,'error':result.normalized_error,'overlap':result.overlap,
                                 'min_eigenvalue':result.min_eigenvalue,'condition':result.condition_number}
                        self.results.append((pose_msg(result.transform),metrics))
                    except Exception as e:self.policy.reject('配准服务失败：'+str(e),now)
                    self.future=None;self.submit_next()
                elif wall-self.query_started>2.:
                    self.future.cancel();self.future=None;self.guesses=[];self.batch=None
                    self.policy.reject('配准计算超时',now)
            radius=self.config['robot']['circumscribed_radius_m']
            was_initialized=self.policy.initialized
            state=self.policy.status(now,radius,.02,self.nav['max_speed_mps'])
            if was_initialized and not self.policy.initialized:
                self.search_started=wall;self.retry_at=0.;self.seed=None
            if not self.policy.initialized and wall-self.search_started>self.p['search_timeout_s']:
                self.retry_at=wall+self.p['retry_s'];self.search_started=self.retry_at
                self.policy.reason='全局定位尚不唯一，保持停车后自动重试'
            if self.future is None and self.batch is None and wall>=self.retry_at and wall-self.last_query>=self.p['period_s']:
                self.last_query=wall;self.start_batch()
            state.update(stamp_ns=self.get_clock().now().nanoseconds,backend='small_gicp',
                         searching=not self.policy.initialized)
            if self.policy.correction is not None:
                tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='navcar_odom'
                tf.header.stamp=(self.get_clock().now()+Duration(seconds=.2)).to_msg()
                p=write_pose(__import__('geometry_msgs.msg',fromlist=['Pose']).Pose(),self.policy.correction)
                tf.transform.translation.x=p.position.x;tf.transform.translation.y=p.position.y;tf.transform.translation.z=p.position.z
                tf.transform.rotation=p.orientation;self.tf.sendTransform(tf)
            # Never emit NaN/Infinity into Web JSON on failed registration.
            state['metrics']={k:(v if not isinstance(v,float) or math.isfinite(v) else None) for k,v in state['metrics'].items()}
            msg=String();msg.data=json.dumps(state,ensure_ascii=False,allow_nan=False);self.status_pub.publish(msg)
            typed=LocalizationStatus();typed.header.frame_id='map';typed.header.stamp=self.get_clock().now().to_msg()
            typed.state={'acquiring':0,'tracking':1,'degraded':2,'lost':3}[state['state']]
            typed.ready=bool(state['ready']);typed.speed_limit_mps=float(state['speed_limit_mps'])
            self.typed_status_pub.publish(typed)
            signature=(state['state'],state['message'])
            if signature!=self.last_message:self.get_logger().info(msg.data);self.last_message=signature
    rclpy.init();node=Localizer()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
