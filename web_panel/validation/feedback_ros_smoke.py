#!/usr/bin/env python3
"""Isolated PTY/ROS test. Never opens the configured physical serial device."""
import argparse
import json
import math
import os
from pathlib import Path
import pty
import select
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import tty
from urllib.request import Request, urlopen

os.environ['ROS_DOMAIN_ID']='84'
os.environ['ROS_LOCALHOST_ONLY']='1'
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from navcar_protocol import frame, STATE, EXTENDED_STATE, ACK, crc16_modbus


def main():
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from shaoshao_hw_interfaces.msg import ChassisFeedback
    from std_msgs.msg import String
    import yaml

    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--revision',type=int,choices=(1,2),default=2)
    revision=parser.parse_args().revision;valid_flags=15 if revision==1 else 11
    master,slave=pty.openpty();tty.setraw(slave)
    path=os.ttyname(slave)
    assert path.startswith('/dev/pts/'),path
    lock=threading.Lock();done=threading.Event()
    state={'flags':valid_flags,'epoch':1,'extended':True,'valid':True}
    commands=[]
    def mcu():
        sequence=0;command_seq=65535;buffer=bytearray();start=time.monotonic()
        while not done.is_set():
            with lock:s=dict(state)
            vy,vx,omega=(.2,.1,.3) if s['valid'] else (math.nan,)*3
            raw=frame(STATE,command_seq,struct.pack('<4f',vy,vx,omega,17.))
            sequence+=1
            if s['extended']:
                raw+=frame(EXTENDED_STATE,command_seq,struct.pack('<BBHIII5f',
                    revision,s['flags'],0,sequence,int((time.monotonic()-start)*1000),s['epoch'],
                    vy,vx,omega,math.pi/2 if revision==1 else 0.,17.))
            os.write(master,raw)
            if select.select([master],[],[],.01)[0]:buffer.extend(os.read(master,4096))
            while len(buffer)>=8:
                if buffer[:2]!=b'\xa5\x5a':del buffer[0];continue
                size=struct.unpack_from('<H',buffer,6)[0]
                if len(buffer)<size+10:break
                packet=bytes(buffer[:size+10]);del buffer[:size+10]
                assert packet[3]==1 and size==16
                assert crc16_modbus(packet[2:-2])==struct.unpack_from('<H',packet,len(packet)-2)[0]
                command_seq=struct.unpack_from('<H',packet,4)[0]
                commands.append(struct.unpack_from('<4f',packet,8))
                os.write(master,frame(ACK,command_seq,b'\0'))
            done.wait(.005)
    worker=threading.Thread(target=mcu,daemon=True)
    rclpy.init(args=[]);node=rclpy.create_node('feedback_smoke_observer')
    raw_samples=[];samples=[];statuses=[]
    node.create_subscription(TwistStamped,'/navcar/chassis_velocity',raw_samples.append,10)
    node.create_subscription(ChassisFeedback,'/hw/chassis_feedback',samples.append,10)
    node.create_subscription(String,'/navcar/chassis_feedback/status',lambda m:statuses.append(json.loads(m.data)),10)
    proc=None
    try:
        with tempfile.TemporaryDirectory(prefix='shaoshao-feedback-') as folder:
            root=Path(folder)
            config=dict(transport='navcar_uart1_v1',state_semantics='wheel_velocity_v2',
                require_extended_feedback_for_navigation=True,serial_port=path,baud=2000000,
                yaw_target_deg=0.,forward_units_per_mps=1.,left_units_per_mps=1.,
                rot_units_per_radps=1.,max_translation_units=.3,max_rot_units=6.)
            (root/'serial.yaml').write_text(yaml.safe_dump(config))
            (root/'robot.yaml').write_text(yaml.safe_dump({'navigation':{'max_speed_mps':.3}}))
            with socket.socket() as sock:sock.bind(('127.0.0.1',0));http_port=sock.getsockname()[1]
            with (root/'web.log').open('w') as log:
                proc=subprocess.Popen([sys.executable,str(Path(__file__).resolve().parents[1]/'navcar_move_web.py'),
                    '--hw-ros','--config',str(root/'serial.yaml'),'--robot-config',str(root/'robot.yaml'),
                    '--bind','127.0.0.1','--port',str(http_port)],stdout=log,stderr=subprocess.STDOUT,
                    env={**os.environ,'SHAOSHAO_HW_RUNTIME':str(root)},start_new_session=True)
                worker.start()
                def until(predicate,timeout=8):
                    deadline=time.monotonic()+timeout
                    while time.monotonic()<deadline:
                        rclpy.spin_once(node,timeout_sec=.02)
                        if predicate():return
                        if proc.poll() is not None:raise AssertionError((root/'web.log').read_text().split('NAVCAR MOVE:')[0])
                    raise AssertionError('feedback smoke timed out')
                until(lambda:(root/'navcar_move.token').exists() and raw_samples and samples and statuses)
                token=(root/'navcar_move.token').read_text().strip()
                def request(endpoint,body=None):
                    headers={'Authorization':'Bearer '+token,'Content-Type':'application/json'}
                    data=json.dumps(body).encode() if body is not None else None
                    with urlopen(Request(f'http://127.0.0.1:{http_port}'+endpoint,data=data,headers=headers),timeout=2) as response:
                        return json.load(response)
                assert not commands,'startup must remain passive'
                msg=samples[-1];raw_msg=raw_samples[-1]
                assert msg.control_allowed and msg.header.frame_id=='navcar_center'
                if revision==1:
                    assert abs(msg.vx_mps+.1)<1e-6 and abs(msg.vy_mps+.2)<1e-6
                else:
                    assert abs(msg.vx_mps-.2)<1e-6 and abs(msg.vy_mps+.1)<1e-6
                    assert msg.gimbal_relative_yaw_rad==0.,'legacy unused field is reserved, not an angle measurement'
                assert raw_msg.header.frame_id=='navcar_chassis' and abs(raw_msg.twist.linear.x-.2)<1e-6
                assert abs(raw_msg.twist.linear.y+.1)<1e-6
                age=(node.get_clock().now().nanoseconds-msg.header.stamp.sec*10**9-msg.header.stamp.nanosec)*1e-9
                assert -.05<=age<=.15,age
                lease=request('/api/navigation/acquire',{})['lease']
                request('/api/navigation/command',dict(lease=lease,seq=1,forward=.05,left=.02))
                until(lambda:any(v[0]>.049 for v in commands))
                with lock:state.update(flags=valid_flags&~1,epoch=2)
                until(lambda:samples and not samples[-1].control_allowed)
                until(lambda:request('/api/state')['control_source']=='manual')
                until(lambda:commands and not any(commands[-1][:3]))
                with lock:state.update(flags=valid_flags,epoch=3)
                until(lambda:request('/api/state')['extended_feedback']['ready'])
                assert request('/api/state')['control_source']=='manual','old navigation lease cannot resume'
                with lock:state.update(valid=False,flags=valid_flags&~2)
                until(lambda:statuses and not statuses[-1]['raw_velocity_valid'])
                value=request('/api/state')
                assert value['connected'] and value['feedback_measurements']['forward_mps'] is None
                assert value['feedback_error']
                with lock:state.update(valid=True,flags=valid_flags)
                until(lambda:statuses and statuses[-1]['navigation_ready'])
                with lock:state.update(extended=False)
                until(lambda:statuses and not statuses[-1]['navigation_ready'])
                value=request('/api/state')
                assert value['connected'] and value['feedback_measurements']['velocity_valid']
                assert '超时' in value['feedback_error']
                print(json.dumps({'ok':True,'wire_revision':revision,'physical_serial_opened':False,
                    'raw_ros_samples':len(raw_samples),'converted_ros_samples':len(samples),
                    'checked':['V2 measurements','frame conversion','ROS timestamps','passive startup',
                        'fake navigation command','takeover revoke','no lease resume','NaN invalidity','extension timeout']},ensure_ascii=False))
    finally:
        if proc is not None and proc.poll() is None:
            os.killpg(proc.pid,signal.SIGTERM)
            try:proc.wait(3)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        done.set()
        if worker.is_alive():worker.join(1)
        os.close(master);os.close(slave)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
