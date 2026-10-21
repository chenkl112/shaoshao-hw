"""HW-inspired global registration and holonomic MPPI behind the Shaoshao Web API."""
import json
import math
from pathlib import Path
import shlex
import subprocess
import sys
import threading
import time
import numpy as np
import yaml

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from shaoshao_hw.config import load
from shaoshao_hw.index import RetrievalIndex
from shaoshao_hw.motion import MotionLimiter,CollisionGrid
from navcar_navigation_legacy import navigation_geometry,localization_quality
from navcar_navigation_legacy import NavigationControl as BaseControl,NavigationBridge as BaseBridge
from navcar_navigation_legacy import build_navigation_config as base_config
from navcar_reference import shift_pose,shift_planar_covariance
from robot_io import sample_monotonic_time


def build_navigation_config(root,maps,initial_pose=None):
    root,maps=Path(root),Path(maps)
    config=load(root/'shaoshao/web_panel/robot.yaml');nav=config['navigation']
    metadata=yaml.safe_load((maps/'site.yaml').read_text(encoding='utf-8'))
    if not math.isclose(metadata.get('resolution',-1),nav['map_resolution_m'],rel_tol=0,abs_tol=1e-8):
        raise ValueError('地图分辨率必须与 map_resolution_m 一致；不能仅修改旧地图 YAML')
    output=base_config(root,maps,initial_pose)
    params=yaml.safe_load((output/'nav2.yaml').read_text(encoding='utf-8'))
    frames=yaml.safe_load((output/'frames.yaml').read_text(encoding='utf-8'))
    frames.update(maps_dir=str(maps.resolve()),robot_radius_m=params['local_costmap']['local_costmap']['ros__parameters']['robot_radius'],
                  footprint_padding_m=.02)
    params.pop('amcl',None)
    speed=nav['max_speed_mps']
    ctrl=params['controller_server']['ros__parameters']
    ctrl.update(odom_duration=.1,speed_limit_topic='/navcar/speed_limit')
    ctrl['FollowPath']={
      'plugin':'nav2_mppi_controller::MPPIController','motion_model':'Omni',
      'time_steps':40,'model_dt':.05,'batch_size':1000,'iteration_count':1,
      'vx_max':speed,'vx_min':-speed,'vy_max':speed,'wz_max':0.,
      # MPPI divides its control regularization by variance: zero noise gives
      # NaN even with a zero angular bound. The motion model still clamps wz=0.
      'vx_std':.2,'vy_std':.2,'wz_std':1e-6,'ax_max':.3,'ax_min':-.3,'ay_max':.3,'ay_min':-.3,
      'az_max':.3,'temperature':.3,'gamma':.015,'prune_distance':2.,
      'transform_tolerance':.3,'visualize':False,'regenerate_noises':False,
      'critics':['ShaoShaoNormCritic','ConstraintCritic','CostCritic','GoalCritic','PathAlignCritic','PathFollowCritic'],
      'ShaoShaoNormCritic':{'enabled':True,'max_speed_mps':speed},
      'ConstraintCritic':{'enabled':True,'cost_power':1,'cost_weight':4.},
      'CostCritic':{'enabled':True,'cost_power':1,'cost_weight':3.81,'critical_cost':253.,
                    'collision_cost':1000000.,'consider_footprint':True,'near_goal_distance':.3},
      'GoalCritic':{'enabled':True,'cost_power':1,'cost_weight':5.,'threshold_to_consider':1.4},
      'PathAlignCritic':{'enabled':True,'cost_power':1,'cost_weight':14.,'threshold_to_consider':.3,
                         'use_path_orientations':False,'trajectory_point_step':2,'offset_from_furthest':5},
      'PathFollowCritic':{'enabled':True,'cost_power':1,'cost_weight':5.,'threshold_to_consider':.3,'offset_from_furthest':5}}
    local=params['local_costmap']['local_costmap']['ros__parameters']
    local.update(width=3,height=3,publish_frequency=10.,track_unknown_space=True,always_send_full_costmap=True)
    # MPPI footprint mode requires an explicit polygon. Encode the padded circle
    # as a circumscribed 64-gon; avoid costmap's axis-wise padding expanding a
    # circle by sqrt(2)*padding at diagonal headings.
    padded=frames['robot_radius_m']+.02
    radius=padded/math.cos(math.pi/64)
    footprint=[[radius*math.cos((i+.5)*2*math.pi/64),radius*math.sin((i+.5)*2*math.pi/64)] for i in range(64)]
    for name in ('local_costmap','global_costmap'):
        cost=params[name][name]['ros__parameters']
        cost['footprint']=json.dumps(footprint);cost['footprint_padding']=0.
    params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']=str(root/'shaoshao/web_panel/navcar_translation.xml')
    (output/'frames.yaml').write_text(yaml.safe_dump(frames,sort_keys=False))
    (output/'nav2.yaml').write_text(yaml.safe_dump(params,sort_keys=False))
    return output


class NavigationControl(BaseControl):
    def reason(self):
        p=self.panel
        if self.error or self.starting or self.stopping or not self.running:return super().reason()
        if not p.nav_localized:return getattr(p.bridge,'localization_info',{}).get('message','等待自动定位')
        return super().reason()

    def _start(self):
        try:
            from ament_index_python.packages import get_package_prefix
            for package in ('hw_registration','shaoshao_hw_runtime','nav2_mppi_controller','pointcloud_to_laserscan'):
                get_package_prefix(package)
            if self.mapping.is_running():self.mapping.request_stop()
            deadline=time.monotonic()+60
            while self.mapping.stopping and time.monotonic()<deadline:time.sleep(.1)
            if self.mapping.error or self.mapping.is_running():raise RuntimeError(self.mapping.error or '建图尚未停止')
            RetrievalIndex(self.mapping.maps_dir/'keyframes',self.mapping.maps_dir/'site.pcd')
            output=build_navigation_config(self.root,self.mapping.maps_dir,self.initial_pose)
            self.frames=yaml.safe_load((output/'frames.yaml').read_text(encoding='utf-8'))
            self.max_speed=load(self.root/'shaoshao/web_panel/robot.yaml')['navigation']['max_speed_mps']
            with self.panel.lock:
                self.panel.nav_localized=False;self.panel.map_pose_time=0.;self.panel.commands.clear()
                self.panel.bridge.reset_hw();self.panel.clear_navigation_plan()
                self.panel.cloud_time=self.panel.odom_time=0.
            setup='source /opt/ros/jazzy/setup.bash\nsource '+shlex.quote(str(self.root/'mapping_install/setup.bash'))+'\n'
            cmd=['ros2','launch',str(self.root/'shaoshao/web_panel/navcar_navigation.launch.py')]
            with (self.root/'navigation.log').open('a') as log:
                self.process=subprocess.Popen(['bash','-c',setup+'exec '+shlex.join(cmd)],stdin=subprocess.DEVNULL,
                    stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            (self.root/'navigation.pid').write_text(str(self.process.pid)+'\n')
            time.sleep(2)
            if not self.running:raise RuntimeError('导航启动失败，请查看 navigation.log')
        except Exception as e:self.error=str(e)
        finally:self.starting=False


class NavigationBridge(BaseBridge):
    def __init__(self,node,panel,root):
        from std_msgs.msg import String
        from nav_msgs.msg import OccupancyGrid
        from nav2_msgs.msg import SpeedLimit
        from nav2_msgs.srv import ManageLifecycleNodes
        from std_srvs.srv import Trigger
        from shaoshao_hw_interfaces.msg import NavCommand
        config=load(Path(root)/'shaoshao/web_panel/robot.yaml')
        self.motion=MotionLimiter(config['navigation']['motion']);self.collision=None
        self.quality_at=0.;self.hw_status={'ready':False,'state':'acquiring','message':'等待自动全局定位','speed_limit_mps':0.}
        self.auto_resume=None;self.resume_phase=None;self._loss_cancel=False;self.manual_epoch=None
        self.stack_ready=False;self.startup_future=None
        self.motion_info={}
        self.startup_client=node.create_client(ManageLifecycleNodes,'/navigation_lifecycle/manage_nodes')
        self.retry_client=node.create_client(Trigger,'/hw/localization/retry')
        self.velocity_pub=node.create_publisher(NavCommand,'/hw/nav_command',10)
        self.speed_pub=node.create_publisher(SpeedLimit,'/navcar/speed_limit',10)
        super().__init__(node,panel,root)
        node.destroy_subscription(self.subs[1]);del self.subs[1]  # no AMCL authority
        self.subs.extend([node.create_subscription(String,'/hw/localization/status',self.quality,10),
                         node.create_subscription(OccupancyGrid,'/local_costmap/costmap',self.local_grid,10)])
        self.status_thread=threading.Thread(target=self.poll_controller,daemon=True);self.status_thread.start()

    def reset_hw(self):
        self.motion.stop();self.collision=None;self.quality_at=0.;self.stack_ready=False
        self.startup_future=None;self.clear_resume()
        self.hw_status={'ready':False,'state':'acquiring','message':'正在自动搜索地图位置','speed_limit_mps':0.}
        self.localization_info=self.hw_status

    def clear_resume(self):self.auto_resume=None;self.resume_phase=None

    def retry_localization(self):
        from std_srvs.srv import Trigger
        p=self.panel
        if not p.navigation_control.running or not self.retry_client.service_is_ready():raise RuntimeError('定位服务尚未就绪')
        with p.lock:
            if p.nav_goal:
                self.auto_resume={**p.nav_goal,'revision':p.saved_map()[0]['revision']}
                self._loss_cancel=True
                try:p.cancel_navigation()
                finally:self._loss_cancel=False
            self.motion.stop();p.nav_localized=False
        self.retry_client.call_async(Trigger.Request())

    def quality(self,msg):
        if not self.panel.navigation_control.running:return
        try:
            value=json.loads(msg.data)
            stamp=value['stamp_ns'];clock=self.node.get_clock().now().nanoseconds
            if type(stamp) is not int or not -.05<=(clock-stamp)*1e-9<=.5:raise ValueError('Stale status')
            if type(value['ready']) is not bool or value['state'] not in ('acquiring','tracking','degraded','lost'):raise ValueError('Bad state')
            limit=value['speed_limit_mps']
            if type(limit) not in (int,float) or not math.isfinite(limit) or not 0<=limit<=.5:raise ValueError('Bad limit')
        except (ValueError,KeyError,TypeError):return
        with self.panel.lock:
            self.hw_status=value;self.quality_at=time.monotonic();self.localization_info=value
            self.panel.nav_localized=value['ready']

    def local_grid(self,msg):
        if msg.header.frame_id!='navcar_odom' or msg.info.width*msg.info.height>1000000:return
        stamp=sample_monotonic_time(msg.header.stamp,self.node.get_clock().now().nanoseconds,time.monotonic(),.5)
        if stamp is None:return
        p,q=msg.info.origin.position,msg.info.origin.orientation
        origin=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        try:grid=CollisionGrid(np.asarray(msg.data).reshape(msg.info.height,msg.info.width),msg.info.resolution,origin,stamp)
        except (ValueError,TypeError):return
        with self.panel.lock:self.collision=grid

    def costmap(self,msg):
        if msg.header.frame_id!='map' or not 0<msg.info.width*msg.info.height<=16000000:return
        p,q=msg.info.origin.position,msg.info.origin.orientation
        data=np.asarray(msg.data)
        with self.panel.lock:self.panel.navigation_costmap={'available':True,'frame':'map',
            'resolution':msg.info.resolution,'width':msg.info.width,'height':msg.info.height,
            'origin':[p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))],
            'blocked':np.flatnonzero(data==99).tolist(),'lethal':np.flatnonzero(data==100).tolist()}

    def odom(self,msg):
        p=self.panel
        sampled=sample_monotonic_time(msg.header.stamp,self.node.get_clock().now().nanoseconds,time.monotonic(),1.)
        if sampled is None or msg.header.frame_id!='navcar_odom':return
        pos,q=msg.pose.pose.position,msg.pose.pose.orientation
        t=msg.twist.twist
        values=[pos.x,pos.y,pos.z,q.x,q.y,q.z,q.w,t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z]
        if not all(math.isfinite(v) for v in values):return
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        reference=p.navigation_control.frames;offset=reference['center_offset']
        pos.x,pos.y=shift_pose(pos.x,pos.y,yaw,offset)
        velocity=np.array([t.linear.x,t.linear.y,t.linear.z])+np.cross([t.angular.x,t.angular.y,t.angular.z],offset)
        t.linear.x,t.linear.y,t.linear.z=velocity.tolist()
        r=np.array(offset);skew=np.array([[0,-r[2],r[1]],[r[2],0,-r[0]],[-r[1],r[0],0]])
        j=np.eye(6);j[:3,3:]=-skew
        msg.twist.covariance=(j@np.asarray(msg.twist.covariance).reshape(6,6)@j.T).reshape(-1).tolist()
        msg.pose.covariance=shift_planar_covariance(list(msg.pose.covariance),yaw,offset)
        msg.child_frame_id=reference['base_frame']
        with p.lock:
            p.pose={'x':pos.x,'y':pos.y,'z':pos.z,'yaw':yaw,'frame':msg.header.frame_id,'reference_offset_xy_m':offset[:2]}
            p.odom_time=sampled
        self.odom_pub.publish(msg)

    def tick(self):
        from nav2_msgs.srv import ManageLifecycleNodes
        from nav2_msgs.msg import SpeedLimit
        p=self.panel
        if not p.navigation_control.running:return
        now=time.monotonic()
        with p.lock:
            if now-self.quality_at>1.:
                p.nav_localized=False;self.localization_info={**self.hw_status,'ready':False,'message':'定位状态更新超时'}
            if p.nav_localized and not self.stack_ready and self.startup_future is None and self.startup_client.service_is_ready():
                request=ManageLifecycleNodes.Request();request.command=0
                self.startup_future=self.startup_client.call_async(request)
            if self.startup_future is not None and self.startup_future.done():
                try:self.stack_ready=bool(self.startup_future.result().success)
                except Exception:self.stack_ready=False
            BaseBridge.tick(self)
            p.nav_server_ready=p.nav_server_ready and self.stack_ready
            limit=SpeedLimit();limit.header.stamp=self.node.get_clock().now().to_msg();limit.percentage=False
            limit.speed_limit=min(p.navigation_control.max_speed,self.hw_status.get('speed_limit_mps',0.) or .05)
            self.speed_pub.publish(limit)
            lost=self.hw_status.get('state') in ('lost','acquiring') or now-self.quality_at>1.
            if lost and p.nav_status in ('active','sending') and self.auto_resume is None:
                self.auto_resume=dict(p.nav_goal) if p.nav_goal else None
                if self.auto_resume:self.auto_resume['revision']=p.saved_map()[0]['revision']
                self._loss_cancel=True
                try:p.cancel_navigation()
                finally:self._loss_cancel=False
                self.motion.stop()
            if self.auto_resume and p.nav_status not in ('active','sending','canceling') and not p.navigation_reason() and self.hw_status.get('state')=='tracking':
                goal=self.auto_resume;revision=p.saved_map()[0]['revision']
                if revision!=goal['revision']:self.clear_resume();return
                if self.resume_phase is None:
                    p.plan_navigation_goal(goal['x'],goal['y'],revision);self.resume_phase=p.plan_serial
                elif p.plan_serial==self.resume_phase and p.nav_plan['status']=='ready':
                    self.clear_resume();p.send_navigation_goal(goal['x'],goal['y'],revision)
                elif p.nav_plan['status']=='failed':
                    self.resume_phase=None;p.nav_message='重新定位成功，原目标路径暂不可通行'

    def poll_controller(self):
        while not self.done.wait(.2):
            try:
                state=self.request('/api/state')
                with self.panel.lock:
                    self.serial_ready=bool(state['connected'] and not state['error']);self.serial_checked=time.monotonic()
                    epoch=state.get('manual_epoch',0)
                    manual_changed=self.manual_epoch is not None and epoch!=self.manual_epoch
                    self.manual_epoch=epoch
                    if manual_changed or (self.lease and state['control_source']!='navigation'):
                        self.clear_resume();self.panel.cancel_navigation();self.motion.stop();self.lease=None
            except Exception:
                self.serial_ready=False

    def output(self):
        from shaoshao_hw_interfaces.msg import NavCommand
        previous=time.monotonic()
        while not self.done.wait(.02):
            p=self.panel;now=time.monotonic();dt=now-previous;previous=now
            with p.lock:
                active=p.nav_status=='active' and p.navigation_control.running and not p.nav_hold and not p.switching
                stamp,values=p.commands.get('auto',(0.,(0.,0.,0.)))
                moving=active and not p.navigation_reason() and now-stamp<.2 and now-self.quality_at<1.
                limit=min(p.navigation_control.max_speed,self.hw_status.get('speed_limit_mps',0.))
                velocity=self.motion.step(values[:2],dt,limit,emergency=not moving)
                pose=p.pose
                safe=(not any(velocity) or (pose and self.collision and self.collision.safe(
                    (pose['x'],pose['y'],pose['yaw']),velocity,p.navigation_control.frames['robot_radius_m']+.02,
                    now,min(self.motion.p['acceleration_mps2'],self.motion.p['deceleration_mps2']),
                    self.motion.p['jerk_mps3'])))
                if not safe:velocity=self.motion.stop()
                self.motion_info={'raw_xy':list(values[:2]),'sent_xy':list(velocity),
                    'moving_allowed':bool(moving),'collision_clear':bool(safe),
                    'raw_age_s':max(0.,now-stamp),'status_age_s':max(0.,now-self.quality_at),
                    'grid_age_s':None if self.collision is None else max(0.,now-self.collision.stamp),
                    'reason':p.navigation_reason() if not moving else ''}
                if self.lease:
                    lease=self.lease
                    self.seq+=1;msg=NavCommand();msg.header.stamp=self.node.get_clock().now().to_msg()
                    msg.header.frame_id=p.navigation_control.frames['base_frame'];msg.lease=lease;msg.seq=self.seq
                    msg.twist.linear.x,msg.twist.linear.y=velocity;msg.twist.angular.z=0.
                    try:self.velocity_pub.publish(msg)
                    except Exception:
                        self.motion.stop();p.nav_hold=True;p.nav_message='速度通道异常，已停止输出'

    def close(self):
        super().close();self.status_thread.join(1)
