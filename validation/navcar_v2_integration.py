#!/usr/bin/env python3
"""UART1 V2 C++/ROS/HTTP/PTY contract test; never opens the vehicle serial port."""
import json,math,os,signal,socket,subprocess,tempfile,threading,time,urllib.request
from pathlib import Path
import numpy as np
import yaml
from ros_integration import NativeMCU,wait_for,SOURCE

def main():
    if os.environ.get('SHAOSHAO_HW_INTEGRATION')!='1' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Use isolated validation environment')
    import rclpy
    from rclpy.node import Node
    from shaoshao_hw_interfaces.msg import NavCommand,SpinCommand,NavcarFeedback,ChassisFeedback
    from ament_index_python.packages import get_package_prefix
    rclpy.init();mcu=NativeMCU();process=None;thread=None
    root=Path(os.environ['SHAOSHAO_HW_RUNTIME']);root.mkdir(parents=True,exist_ok=True)
    with socket.socket() as sock:sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
    robot=root/'robot.yaml';robot.write_text((SOURCE/'web_panel/robot.yaml').read_text());(root/'active_robot.yaml').write_text(robot.read_text())
    config=yaml.safe_load((SOURCE/'web_panel/serial.navcar.example.yaml').read_text());config['serial_port']=mcu.port
    serial=root/'serial.yaml';serial.write_text(yaml.safe_dump(config))
    class Client(Node):
        def __init__(self):
            super().__init__('navcar_v2_contract');self.states=[];self.fabricated=[];self.lease='';self.nav=True;self.spin=True;self.seq=0;self.spin_seq=0
            self.nav_pub=self.create_publisher(NavCommand,'/hw/nav_command',1);self.spin_pub=self.create_publisher(SpinCommand,'/hw/spin_command',1)
            self.create_subscription(NavcarFeedback,'/hw/navcar/feedback',self.states.append,10)
            self.create_subscription(ChassisFeedback,'/hw/chassis_feedback',self.fabricated.append,10)
            self.create_timer(.02,self.tick)
        def tick(self):
            if not self.lease:return
            stamp=self.get_clock().now().to_msg()
            if self.nav:
                self.seq+=1;m=NavCommand();m.header.stamp=stamp;m.header.frame_id='navcar_center';m.seq=self.seq;m.lease=self.lease;m.twist.linear.x=.2;m.twist.linear.y=.1;self.nav_pub.publish(m)
            if self.spin:
                self.spin_seq+=1;m=SpinCommand();m.header.stamp=stamp;m.header.frame_id='navcar_center';m.seq=self.spin_seq;m.lease=self.lease;m.omega_radps=.7;self.spin_pub.publish(m)
    client=Client();thread=threading.Thread(target=rclpy.spin,args=(client,),daemon=True);thread.start()
    def api(path,body=None):
        token=(root/'navcar_move.token').read_text().strip();req=urllib.request.Request(f'http://127.0.0.1:{port}'+path,
            data=None if body is None else json.dumps(body).encode(),headers={'Authorization':'Bearer '+token,'Content-Type':'application/json'})
        with urllib.request.urlopen(req,timeout=2) as response:return json.load(response)
    def last():
        with mcu.lock:return mcu.commands[-1] if mcu.commands else (0,0,0,0,0)
    try:
        binary=Path(get_package_prefix('shaoshao_hw_runtime'))/'lib/shaoshao_hw_runtime/hw_move_web'
        legacy=yaml.safe_load((SOURCE/'web_panel/serial.navcar.v1.example.yaml').read_text());legacy['serial_port']='/dev/protocol-mismatch-must-not-open'
        mismatch=root/'mismatch.yaml';mismatch.write_text(yaml.safe_dump(legacy))
        guard=subprocess.run([str(binary),'--runtime',str(root),'--config',str(mismatch),'--robot-config',str(robot)],capture_output=True,text=True,timeout=3)
        assert guard.returncode!=0 and 'protocol mismatch' in guard.stderr,'V1/V2 mismatch must fail before opening hardware'
        with (root/'serial.log').open('w') as log:
            process=subprocess.Popen([str(binary),'--runtime',str(root),'--config',str(serial),'--robot-config',str(robot),'--bind','127.0.0.1','--port',str(port),'--hw-ros'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            wait_for(lambda:api('/api/state')['connected'] and len(client.states)>5,10,'Receive V2 raw feedback')
            before=client.states[-3:];assert len({m.last_command_seq for m in before})==1
            assert before[-1].reception_seq>before[0].reception_seq
            assert abs(before[-1].gimbal_yaw_start_rad-math.radians(17))<1e-6
            client.lease=api('/api/navigation/acquire',{})['lease']
            wait_for(lambda:abs(last()[1]-.2)<1e-5 and abs(last()[2]-.1)<1e-5 and abs(last()[3]-.7)<1e-5,8,'V2 SI translation and independent positive-CCW spin')
            wait_for(lambda:math.hypot(client.states[-1].forward_mps,client.states[-1].left_mps)>.15,3,'Wheel-derived body measurement')
            measured=client.states[-1]
            with mcu.lock:v=mcu.velocity.copy()
            c,s=math.cos(.4),math.sin(.4);expected=np.array([c*v[0]-s*v[1],s*v[0]+c*v[1]])
            assert np.linalg.norm([measured.forward_mps-expected[0],measured.left_mps-expected[1]])<.03
            assert abs(measured.chassis_omega_radps)<1e-6 and last()[3]>.69,'State must remain a measurement when command is nonzero'
            client.nav=False
            wait_for(lambda:abs(last()[1])+abs(last()[2])<1e-6 and last()[3]>.69,.42,'Spin heartbeat must not renew translation watchdog')
            client.nav=True;client.spin=False
            wait_for(lambda:last()[1]>.19 and abs(last()[3])<1e-6,.42,'Translation heartbeat must not renew spin watchdog')
            client.spin=True;wait_for(lambda:last()[3]>.69,2,'Independent spin resumes')
            mcu.invalid_wheels=True
            wait_for(lambda:not client.states[-1].velocity_valid and client.states[-1].yaw_valid,2,'NaN velocity with independently valid yaw')
            wait_for(lambda:abs(last()[1])+abs(last()[2])+abs(last()[3])<1e-6,2,'Invalid wheel feedback stops all outgoing motion')
            state=api('/api/state');assert state['measurements']['forward_body_mps'] is None and state['yaw_actual_deg']==17 and state['feedback_targets'] is None
            assert state['measurements']['sample_timestamp_available'] is False and state['measurements']['control_mode_available'] is False
            assert not client.fabricated,'V2 raw data must not fabricate synchronized authorized lidar-frame feedback'
            client.lease='';mcu.invalid_wheels=False
            result={'protocol_commit':'43333c533538490c1e2262a63a004cde088d7f79','body_velocity_error_mps':float(np.linalg.norm([measured.forward_mps-expected[0],measured.left_mps-expected[1]])),
                'raw_feedback_messages':len(client.states),'checks':['V1/V2 mismatch fails before opening hardware','V2 100 Hz repeated command sequence','SI and CCW wire mapping','body-frame measurements differ from targets','independent translation/spin watchdogs','NaN velocity invalidates state while yaw stays valid','JSON invalid numbers are null','no fabricated ChassisFeedback or hardware authority']}
            (root/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
    finally:
        if process and process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        rclpy.shutdown();thread.join(2);client.destroy_node();mcu.close()
if __name__=='__main__':main()
