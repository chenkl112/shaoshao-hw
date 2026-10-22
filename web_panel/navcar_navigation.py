"""Indexed localization and HW FDDP control behind the Shaoshao Web API."""
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
from shaoshao_hw.chassis import Feedback,FeedbackGate
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
    ctrl.update(odom_duration=.1,speed_limit_topic='/navcar/speed_limit',enable_stamped_cmd_vel=True)
    ctrl['FollowPath']={
      'plugin':'hw_omni_controller::Controller','max_speed_mps':speed,
      'acceleration_mps2':min(nav['motion']['acceleration_mps2'],nav['motion']['deceleration_mps2']),
      'jerk_mps3':nav['motion']['jerk_mps3'],
      'collision_radius_m':frames['robot_radius_m']+.02,**nav['mpc']}
    ctrl['goal_checker'].update(plugin='nav2_controller::StoppedGoalChecker',
        trans_stopped_velocity=.03,rot_stopped_velocity=100.)
    local=params['local_costmap']['local_costmap']['ros__parameters']
    local.update(global_frame='map',width=4,height=4,publish_frequency=10.,
                 track_unknown_space=True,always_send_full_costmap=True)
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
    params['hw_obstacle_predictor']={'ros__parameters':{
        **nav['obstacles'],'map_path':str(maps/'site.pcd'),
        'lidar_height_m':float(config['robot']['lidar_height_m'])}}
    params['hw_command_relay']={'ros__parameters':{
        'max_speed_mps':speed,'deceleration_mps2':min(nav['motion']['acceleration_mps2'],nav['motion']['deceleration_mps2']),
        'jerk_mps3':nav['motion']['jerk_mps3'],'collision_radius_m':frames['robot_radius_m']+.02,
        'feedback_timeout_s':nav['mpc']['feedback_timeout_s']}}
    (output/'frames.yaml').write_text(yaml.safe_dump(frames,sort_keys=False))
    (output/'nav2.yaml').write_text(yaml.safe_dump(params,sort_keys=False))
    return output


class NavigationControl(BaseControl):
    def reason(self):
        p=self.panel
        if self.error or self.starting or self.stopping or not self.running:return super().reason()
        if not p.nav_localized:return getattr(p.bridge,'localization_info',{}).get('message','等待自动定位')
        if p.bridge and not p.bridge.chassis_gate.ready(p.bridge.node.get_clock().now().nanoseconds*1e-9):
            return p.bridge.chassis_gate.reason(p.bridge.node.get_clock().now().nanoseconds*1e-9)
        return super().reason()

    def _start(self):
        try:
            from ament_index_python.packages import get_package_prefix
            for package in ('hw_registration','shaoshao_hw_runtime','hw_omni_controller','pointcloud_to_laserscan'):
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
        from geometry_msgs.msg import TwistStamped
        from nav2_msgs.msg import SpeedLimit
        from nav2_msgs.srv import ManageLifecycleNodes
        from std_srvs.srv import Trigger
        from shaoshao_hw_interfaces.msg import NavigationSession
        from shaoshao_hw_interfaces.msg import ChassisFeedback
        config=load(Path(root)/'shaoshao/web_panel/robot.yaml')
        self.chassis_gate=FeedbackGate(config['navigation']['mpc']['feedback_timeout_s'])
        self.quality_at=0.;self.hw_status={'ready':False,'state':'acquiring','message':'等待自动全局定位','speed_limit_mps':0.}
        self.auto_resume=None;self.resume_phase=None;self._loss_cancel=False;self.manual_epoch=None
        self.stack_ready=False;self.startup_future=None
        self.motion_info={}
        self.startup_client=node.create_client(ManageLifecycleNodes,'/navigation_lifecycle/manage_nodes')
        self.retry_client=node.create_client(Trigger,'/hw/localization/retry')
        self.session_pub=node.create_publisher(NavigationSession,'/hw/navigation_session',1)
        self.speed_pub=node.create_publisher(SpeedLimit,'/navcar/speed_limit',10)
        super().__init__(node,panel,root)
        node.destroy_subscription(self.subs[0])
        self.subs[0]=node.create_subscription(TwistStamped,'/navcar/cmd_vel_auto',self.stamped_command,1)
        node.destroy_subscription(self.subs[1]);del self.subs[1]  # no AMCL authority
        self.subs.extend([node.create_subscription(String,'/hw/localization/status',self.quality,10),
                         node.create_subscription(ChassisFeedback,'/hw/chassis_feedback',self.chassis_feedback,10),
                         node.create_subscription(String,'/hw/command/status',self.command_status,1)])
        self.status_thread=threading.Thread(target=self.poll_controller,daemon=True);self.status_thread.start()

    def reset_hw(self):
        self.quality_at=0.;self.stack_ready=False
        self.startup_future=None;self.clear_resume()
        self.hw_status={'ready':False,'state':'acquiring','message':'正在自动搜索地图位置','speed_limit_mps':0.}
        self.localization_info=self.hw_status

    def clear_resume(self):self.auto_resume=None;self.resume_phase=None

    def chassis_feedback(self,msg):
        now=self.node.get_clock().now().nanoseconds*1e-9
        sample=Feedback(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,msg.seq,msg.authority_epoch,
            msg.control_allowed,msg.vx_mps,msg.vy_mps,msg.chassis_omega_radps,msg.gimbal_relative_yaw_rad,msg.header.frame_id)
        with self.panel.lock:
            try:changed=self.chassis_gate.update(sample,now)
            except ValueError:changed=True
            if changed or not self.chassis_gate.ready(now):
                self.clear_resume();self.panel.commands.clear()
                if self.panel.nav_status in ('active','sending'):
                    self.panel.cancel_navigation()

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
            p.commands.clear();p.nav_localized=False
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

    def stamped_command(self,msg):
        stamp=sample_monotonic_time(msg.header.stamp,self.node.get_clock().now().nanoseconds,time.monotonic(),.2)
        if stamp is not None and msg.header.frame_id=='navcar_center':BaseBridge.command(self,msg.twist)

    def command_status(self,msg):
        try:
            value=json.loads(msg.data)
            age=(self.node.get_clock().now().nanoseconds-value['stamp_ns'])*1e-9
            if not -.05<=age<=.5:return
        except (ValueError,KeyError,TypeError):return
        with self.panel.lock:self.motion_info=value

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
        # Nav2's stopped-goal checker must see actual chassis translation too.
        # Keep the lidar's angular rate: chassis spin is an independent channel.
        feedback=getattr(self,'chassis_gate',None)
        if feedback and feedback.ready(self.node.get_clock().now().nanoseconds*1e-9):
            t.linear.x=feedback.sample.vx;t.linear.y=feedback.sample.vy
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
                p.commands.clear()
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
                        self.clear_resume();self.panel.cancel_navigation();self.lease=None
            except Exception:
                self.serial_ready=False

    def output(self):
        from shaoshao_hw_interfaces.msg import NavigationSession
        # Task/lease heartbeat only. C++ owns velocity gating and 50 Hz output.
        while not self.done.wait(.05):
            p=self.panel
            with p.lock:
                if not self.lease:continue
                msg=NavigationSession();msg.header.stamp=self.node.get_clock().now().to_msg()
                msg.header.frame_id='navcar_center';msg.lease=self.lease
                sample=self.chassis_gate.sample
                msg.authority_epoch=sample.authority_epoch if sample else 0
                msg.enabled=bool(p.nav_status=='active' and p.navigation_control.running and
                    not p.nav_hold and not p.switching and not p.navigation_reason())
                self.session_pub.publish(msg)

    def close(self):
        super().close();self.status_thread.join(1)
