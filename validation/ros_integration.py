#!/usr/bin/env python3
"""Isolated GICP/FDDP/prediction/Web test with explicit synthetic measured feedback."""
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import threading
import time
import urllib.request
import numpy as np

SOURCE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(SOURCE));sys.path.insert(0,str(SOURCE/'web_panel'));sys.path.insert(0,str(SOURCE/'validation'))
from fixtures import make_bundle,sensor_pose,ray_endpoints
from shaoshao_hw.geometry import matrix,transform
from shaoshao_hw.ros_points import encode
from navcar_protocol import frame,crc16_modbus,ACK,STATE

class NativeMCU:
    def __init__(self):
        import pty,tty
        self.master,self.slave=pty.openpty();tty.setraw(self.slave);os.set_blocking(self.master,False)
        self.port=os.ttyname(self.slave);self.closed=threading.Event();self.lock=threading.Lock()
        self.velocity=np.zeros(2);self.position=np.zeros(2);self.target=np.zeros(2);self.commands=[]
        self.command_at=0.;self.seq=0;self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
    def run(self):
        buffer=bytearray();previous=time.monotonic();last_state=0.
        while not self.closed.wait(.002):
            now=time.monotonic();dt=now-previous;previous=now
            try:buffer.extend(os.read(self.master,4096))
            except BlockingIOError:pass
            while len(buffer)>=10:
                start=buffer.find(b'\xa5\x5a')
                if start<0:buffer.clear();break
                del buffer[:start]
                if len(buffer)<10:break
                version,kind,seq,size=struct.unpack_from('<BBHH',buffer,2)
                if len(buffer)<10+size:break
                packet=bytes(buffer[:10+size]);del buffer[:10+size]
                if crc16_modbus(packet[2:-2])!=struct.unpack('<H',packet[-2:])[0]:continue
                if kind==1 and size==16:
                    native=struct.unpack('<4f',packet[8:-2]);self.seq=seq;self.command_at=now
                    with self.lock:
                        self.target=np.array([native[0],-native[1]])
                        self.commands.append((now,*self.target,native[2],native[3]))
                    os.write(self.master,frame(ACK,seq,b'\x00'))
            with self.lock:
                target=self.target if now-self.command_at<.3 else np.zeros(2)
                self.velocity+=(target-self.velocity)*min(1.,dt/.12)
                self.position+=self.velocity*dt
            if now-last_state>.04:
                os.write(self.master,frame(STATE,self.seq,struct.pack('<4f',self.target[0],-self.target[1],0,17)));last_state=now
    def close(self):
        self.closed.set();self.thread.join(1);os.close(self.master);os.close(self.slave)

def wait_for(test,seconds,message):
    deadline=time.monotonic()+seconds;last=None
    while time.monotonic()<deadline:
        try:
            result=test()
            if result:return result
        except Exception as e:last=str(e)
        time.sleep(.1)
    raise AssertionError(message+(': '+last if last else ''))

def main():
    if os.environ.get('SHAOSHAO_HW_INTEGRATION')!='1' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Use the isolated validation container')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry,OccupancyGrid
    from geometry_msgs.msg import TwistStamped
    from sensor_msgs.msg import PointCloud2
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster
    from shaoshao_hw_interfaces.msg import ChassisFeedback
    import yaml
    runtime=Path(os.environ['SHAOSHAO_HW_RUNTIME']);maps=runtime/'synthetic_map'
    points=make_bundle(maps)
    (runtime/'mapping_runtime').mkdir(exist_ok=True)
    mapping={'/**':{'ros__parameters':yaml.safe_load((SOURCE/'sentry_bringup/params/fast_lio_mapping_param.yaml').read_text())['/**']['ros__parameters']}}
    mp=mapping['/**']['ros__parameters'];mp['mapping']['extrinsic_T']=[-.011,-.02329,.04412]
    mp['mapping']['extrinsic_R']=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
    (runtime/'mapping_runtime/fast_lio.yaml').write_text(yaml.safe_dump(mapping))
    (runtime/'current_mapping_dir').write_text(str(maps));(runtime/'mapping.pid').write_text('0')
    mcu=NativeMCU();cfg=yaml.safe_load((SOURCE/'web_panel/serial.navcar.example.yaml').read_text())
    cfg['serial_port']=mcu.port
    serial_config=runtime/'synthetic_serial.yaml';serial_config.write_text(yaml.safe_dump(cfg))
    processes=[];logs=[];rclpy.init()
    truth=matrix([.6,.5,0,0,0,math.sin(.3/2),math.cos(.3/2)])
    world_odom_points=transform(points,np.linalg.inv(truth))
    class Sensors(Node):
        def __init__(self):
            super().__init__('synthetic_measurements')
            self.cloud=self.create_publisher(PointCloud2,'/cloud_registered',qos_profile_sensor_data)
            self.body=self.create_publisher(PointCloud2,'/cloud_registered_body',qos_profile_sensor_data)
            self.odom=self.create_publisher(Odometry,'/navcar/state_estimation',10)
            self.feedback=self.create_publisher(ChassisFeedback,'/hw/chassis_feedback',10)
            self.feedback_seq=0;self.authority_epoch=1;self.control_allowed=True
            self.tf=TransformBroadcaster(self);self.enabled=True
            self.last_grid=None;self.raw_commands=[]
            self.create_subscription(OccupancyGrid,'/local_costmap/costmap',self.grid,10)
            self.create_subscription(TwistStamped,'/navcar/cmd_vel_auto',lambda m:self.raw_commands.append([m.twist.linear.x,m.twist.linear.y,m.twist.angular.z]),10)
            self.create_timer(.1,self.tick)
            # Independent of LIO availability: localization loss should retain
            # the goal, while actual hardware/manual takeover should revoke it.
            self.create_timer(.02,self.actual_feedback)
        def grid(self,msg):self.last_grid=msg
        def actual_feedback(self):
            with mcu.lock:velocity=mcu.velocity.copy()
            msg=ChassisFeedback();msg.header.frame_id='navcar_center';msg.header.stamp=self.get_clock().now().to_msg()
            self.feedback_seq+=1;msg.seq=self.feedback_seq;msg.authority_epoch=self.authority_epoch
            msg.control_allowed=self.control_allowed;msg.vx_mps=float(velocity[0]);msg.vy_mps=float(velocity[1])
            self.feedback.publish(msg)
        def tick(self):
            if not self.enabled:return
            with mcu.lock:xy=mcu.position.copy();velocity=mcu.velocity.copy()
            now=self.get_clock().now().to_msg()
            self.cloud.publish(encode(world_odom_points,'navcar_odom',now))
            sensor_map=truth@sensor_pose(*xy)
            lidar=ray_endpoints(sensor_map)
            body=lidar+np.array([-.011,-.02329,.04412])
            self.body.publish(encode(body,'navcar_imu',now))
            msg=Odometry();msg.header.frame_id='navcar_odom';msg.header.stamp=now;msg.child_frame_id='navcar_imu'
            msg.pose.pose.position.x=float(xy[0]);msg.pose.pose.position.y=float(xy[1]);msg.pose.pose.orientation.w=1.
            msg.twist.twist.linear.x=float(velocity[0]);msg.twist.twist.linear.y=float(velocity[1])
            self.odom.publish(msg)
            tf=TransformStamped();tf.header=msg.header;tf.child_frame_id='navcar_imu'
            tf.transform.translation.x=float(xy[0]);tf.transform.translation.y=float(xy[1]);tf.transform.rotation.w=1.
            self.tf.sendTransform(tf)
    sensors=Sensors();spin=threading.Thread(target=rclpy.spin,args=(sensors,),daemon=True);spin.start()
    def start(name,args):
        log=(runtime/(name+'.log')).open('w');logs.append(log)
        p=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,
            env={**os.environ,'SCURM_WEB_TOKEN':'integration-only'});processes.append(p);return p
    def api(port,path,body=None):
        token='integration-only' if port==8080 else (runtime/'navcar_move.token').read_text().strip()
        req=urllib.request.Request(f'http://127.0.0.1:{port}'+path,
            data=None if body is None else json.dumps(body).encode(),
            headers={'Authorization':'Bearer '+token,'Content-Type':'application/json'})
        with urllib.request.urlopen(req,timeout=2) as response:return json.load(response)
    try:
        start('move',['python3',str(SOURCE/'web_panel/navcar_move_web.py'),'--hw-ros','--config',str(serial_config),
                      '--robot-config',str(SOURCE/'web_panel/robot.yaml')])
        wait_for(lambda:api(8081,'/api/state')['connected'],10,'Native MCU connection')
        start('web',['python3',str(runtime/'lidar_mapping_web.py'),'--maps-dir',str(maps)])
        wait_for(lambda:api(8080,'/api/state'),10,'Web startup')
        api(8080,'/api/mode',{'mode':'navigation'})
        state=wait_for(lambda:(s if (s:=api(8080,'/api/state'))['navigation']['ready'] else None),50,'Global registration / Nav2 activation')
        recovered=state['navigation']['pose']
        expected=transform(np.array([[.00318,-.02329,0.]]),truth)[0]
        assert np.linalg.norm([recovered['x']-expected[0],recovered['y']-expected[1]])<.06,(recovered,expected)
        info=api(8080,'/api/map');target={'x':float(expected[0]+.6),'y':float(expected[1]+.25),'map_revision':info['revision']}
        api(8080,'/api/navigation/plan',target)
        wait_for(lambda:api(8080,'/api/state')['navigation']['plan']['status']=='ready',10,'Path preview')
        api(8080,'/api/navigation/goal',target)
        wait_for(lambda:any(math.hypot(x,y)>.02 for _,x,y,*_ in mcu.commands),20,'FDDP -> leased ROS -> synthetic native UART motion')
        time.sleep(2)
        sensors.enabled=False;failure_at=time.monotonic()
        wait_for(lambda:api(8080,'/api/state')['navigation'].get('auto_resume_pending'),8,'Lost localization retains goal')
        time.sleep(.4)
        assert mcu.commands and math.hypot(mcu.commands[-1][1],mcu.commands[-1][2])<1e-5
        sensors.enabled=True
        wait_for(lambda:api(8080,'/api/state')['navigation']['status']=='active' and not api(8080,'/api/state')['navigation']['auto_resume_pending'],45,'Re-localize and resume goal')
        resumed_at=time.monotonic()
        wait_for(lambda:any(t>resumed_at and math.hypot(x,y)>.02 for t,x,y,*_ in mcu.commands),10,'Resumed native motion')
        # A takeover while localization is lost must defeat deferred auto-resume.
        sensors.enabled=False
        wait_for(lambda:api(8080,'/api/state')['navigation'].get('auto_resume_pending'),8,'Second loss retains target')
        epoch=api(8081,'/api/state')['manual_epoch']
        sensors.authority_epoch+=1;sensors.control_allowed=False
        api(8081,'/api/command',dict(action='stop',strength=.1,speed_unit='m/s',client='integration-manual',seq=1))
        wait_for(lambda:api(8081,'/api/state')['manual_epoch']>epoch,5,'Manual takeover')
        time.sleep(.5)
        assert not api(8080,'/api/state')['navigation']['auto_resume_pending']
        sensors.enabled=True
        wait_for(lambda:api(8080,'/api/state')['navigation']['localized'],45,'Localization after manual takeover')
        time.sleep(.5)
        state=api(8080,'/api/state')['navigation']
        assert state['status'] not in ('active','sending') and not state['auto_resume_pending']
        assert math.hypot(mcu.commands[-1][1],mcu.commands[-1][2])<1e-5
        assert max(math.hypot(c[1],c[2]) for c in mcu.commands)<=float(yaml.safe_load((SOURCE/'web_panel/robot.yaml').read_text())['navigation']['max_speed_mps'])+1e-5
        result={'global_pose_error_m':float(np.linalg.norm([recovered['x']-expected[0],recovered['y']-expected[1]])),
                'native_commands':len(mcu.commands),'max_command_speed_mps':max(math.hypot(c[1],c[2]) for c in mcu.commands),
                'checks':['automatic unknown-pose GICP','FDDP activation','time-indexed predictions','synthetic measured feedback','Web preview/goal','native CRC/ACK transport',
                          'localization loss stops','automatic replan/resume','manual takeover revokes lease']}
        (runtime/'integration-result.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result),flush=True)
    except Exception:
        debug={}
        try:debug['state']=api(8080,'/api/state')
        except Exception:pass
        debug['raw_commands']=sensors.raw_commands[-10:]
        if sensors.last_grid:
            from scipy.ndimage import distance_transform_edt
            g=sensors.last_grid;a=np.asarray(g.data).reshape(g.info.height,g.info.width)
            d=distance_transform_edt((a>=0)&(a<100))*g.info.resolution
            with mcu.lock:xy=mcu.position.copy()+[.00318,-.02329]
            ix=int((xy[0]-g.info.origin.position.x)/g.info.resolution)
            iy=int((xy[1]-g.info.origin.position.y)/g.info.resolution)
            debug['local_grid']={'counts':{str(k):int(v) for k,v in zip(*np.unique(a,return_counts=True))},
                'center_value':int(a[iy,ix]),'center_clearance':float(d[iy,ix])}
        (runtime/'integration-debug.json').write_text(json.dumps(debug,ensure_ascii=False,indent=2))
        raise
    finally:
        for p in reversed(processes):
            if p.poll() is None:
                try:os.killpg(p.pid,signal.SIGINT);p.wait(timeout=8)
                except (ProcessLookupError,subprocess.TimeoutExpired):
                    try:os.killpg(p.pid,signal.SIGKILL)
                    except ProcessLookupError:pass
        sensors.destroy_node();rclpy.shutdown();spin.join(2);mcu.close()
        for f in logs:f.close()

if __name__=='__main__':main()
