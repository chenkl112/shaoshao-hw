#!/usr/bin/env python3
"""Real FAST-LIO + recorder + map-save chain using synthetic IMU/CustomMsg."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import numpy as np
import yaml

SOURCE=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(SOURCE),str(SOURCE/'validation')]
from fixtures import make_scene
from shaoshao_hw.index import RetrievalIndex


def main():
    if os.environ.get('SHAOSHAO_HW_INTEGRATION')!='1' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Use the isolated validation container')
    import rclpy
    from sensor_msgs.msg import Imu,PointCloud2
    from nav_msgs.msg import Odometry
    from livox_ros_driver2.msg import CustomMsg,CustomPoint
    from std_srvs.srv import Trigger
    runtime=Path(os.environ['SHAOSHAO_HW_RUNTIME'])
    runtime.mkdir(parents=True,exist_ok=True)
    (runtime/'mapping_runtime').mkdir(exist_ok=True)
    maps=runtime/'map';maps.mkdir(exist_ok=True)
    (runtime/'current_mapping_dir').write_text(str(maps))
    if not (runtime/'shaoshao').exists():(runtime/'shaoshao').symlink_to(SOURCE,target_is_directory=True)
    config=yaml.safe_load((SOURCE/'sentry_bringup/params/fast_lio_mapping_param.yaml').read_text())
    p=config['/**']['ros__parameters']
    p['mapping'].update(extrinsic_T=[-.011,-.02329,.04412],extrinsic_est_en=False)
    p['preprocess']['blind']=.2
    p['common']['send_odom_base_tf']=False
    p['point_filter_num']=1;p['filter_size_surf']=.1;p['filter_size_map']=.15
    p['map_file_path']=str(maps/'site.pcd')
    p['map_filter'].update(static_resolution=.025,dynamic_enabled=False)
    (runtime/'mapping_runtime/fast_lio.yaml').write_text(yaml.safe_dump(config))
    rclpy.init();node=rclpy.create_node('synthetic_lio_input')
    imu=node.create_publisher(Imu,'/livox/imu',100)
    cloud=node.create_publisher(CustomMsg,'/livox/lidar',10)
    outputs=[];body=[]
    node.create_subscription(Odometry,'/state_estimation',outputs.append,100)
    node.create_subscription(PointCloud2,'/cloud_registered_body',lambda m:body.append(m.width*m.height),10)
    log=(runtime/'mapping-validation.log').open('w')
    process=subprocess.Popen(['ros2','launch',str(SOURCE/'deployment/mapping_runtime/mapping.launch.py')],
                             stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    points=make_scene();offset=np.array([-.011,-.02329,.04412])
    # Known continuous trajectory: rest -> acceleration -> 0.08 m/s cruise.
    def position(t):
        a=max(0.,t-3.)
        return .025*a*a if a<=1.6 else .064+.08*(a-1.6)
    start=time.monotonic();last_scan=-1;sent=0
    try:
        while time.monotonic()-start<10:
            t=time.monotonic()-start
            stamp=node.get_clock().now().to_msg()
            message=Imu();message.header.stamp=stamp;message.header.frame_id='livox_frame'
            message.linear_acceleration.z=9.81
            message.linear_acceleration.x=.05 if 3<t<4.6 else 0.
            imu.publish(message)
            scan=int(t*10)
            if scan!=last_scan and t>.7:
                last_scan=scan
                message=CustomMsg();message.header.stamp=stamp;message.header.frame_id='livox_frame'
                message.timebase=stamp.sec*10**9+stamp.nanosec
                sample=[]
                for i,point in enumerate(points):
                    dt=.099*i/max(1,len(points)-1)
                    local=point-offset-[position(t+dt),0,0]
                    q=CustomPoint();q.x,q.y,q.z=map(float,local);q.reflectivity=80
                    q.line=i%4;q.tag=0;q.offset_time=int(dt*1e9);sample.append(q)
                message.points=sample;message.point_num=len(sample);cloud.publish(message);sent+=1
            rclpy.spin_once(node,timeout_sec=.001)
            time.sleep(.003)
        assert len(outputs)>20 and body and min(body)>500,(len(outputs),body[-5:])
        recent=outputs[-15:]
        velocities=np.array([[m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.linear.z] for m in recent])
        assert np.isfinite(velocities).all()
        speed=float(np.median(velocities[:,0]))
        assert .04<speed<.12,speed
        for msg in recent:
            for values in (msg.pose.covariance,msg.twist.covariance):
                c=np.asarray(values).reshape(6,6)
                assert np.isfinite(c).all() and np.allclose(c,c.T,atol=1e-7)
                assert np.linalg.eigvalsh(c).min()>-1e-7
            assert msg.twist.covariance[0]>0
        def call(service):
            client=node.create_client(Trigger,service)
            assert client.wait_for_service(timeout_sec=5),service
            future=client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node,future,timeout_sec=30)
            assert future.done() and future.result().success,(service,future.result())
            return future.result().message
        saved=call('/map_save');indexed=call('/hw/finalize_index')
        index=RetrievalIndex(maps/'keyframes',maps/'site.pcd')
        result={'synthetic':True,'scans_sent':sent,'lio_outputs':len(outputs),
                'cruise_velocity_mps':speed,'expected_velocity_mps':.08,
                'map_save':saved,'index_finalize':indexed,'checks':['native LIO velocity','finite PSD covariance',
                'body cloud','actual ROS recorder','PCD save','static index/hash validation']}
        (runtime/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
    finally:
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        log.close()

if __name__=='__main__':main()
