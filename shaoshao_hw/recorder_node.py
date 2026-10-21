from pathlib import Path
import numpy as np
import yaml
from .config import load,RUNTIME
from .geometry import matrix
from .index import IndexWriter
from .ros_points import decode

def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer,TransformListener
    class Recorder(Node):
        def __init__(self):
            super().__init__('hw_index_recorder')
            config=load(self.declare_parameter('config_path','').value or None)
            self.maps=Path(self.declare_parameter('maps_dir','').value)
            lio=yaml.safe_load((RUNTIME/'mapping_runtime/fast_lio.yaml').read_text(encoding='utf-8'))['/**']['ros__parameters']
            self.offset=np.asarray(lio['mapping']['extrinsic_T'])
            self.writer=IndexWriter(self.maps/'keyframes',config['navigation']['index'])
            self.tf=Buffer();self.listener=TransformListener(self.tf,self)
            self.latest=None;self.last_stamp=None;self.finalized=False
            self.create_subscription(PointCloud2,'/cloud_registered_body',self.receive,qos_profile_sensor_data)
            self.create_timer(.1,self.tick)
            self.create_service(Trigger,'/hw/finalize_index',self.finalize)
        def receive(self,msg):
            if not self.finalized:self.latest=msg
        def tick(self):
            msg=self.latest
            if msg is None or self.finalized:return
            stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            if stamp==self.last_stamp:return
            try:
                tf=self.tf.lookup_transform('odom',msg.header.frame_id,Time.from_msg(msg.header.stamp))
                t,q=tf.transform.translation,tf.transform.rotation
                sensor_pose=matrix([t.x,t.y,t.z,q.x,q.y,q.z,q.w])
                offset=self.offset
                sensor_pose[:3,3]+=sensor_pose[:3,:3]@offset
                points=decode(msg)-offset
            except Exception:return
            self.last_stamp=stamp
            self.writer.add(points,sensor_pose,stamp)
        def finalize(self,request,response):
            try:
                self.tick();self.writer.finalize(self.maps/'site.pcd');self.finalized=True
                response.success=True;response.message=f'Indexed {len(self.writer.frames)} static keyframes'
            except Exception as e:response.success=False;response.message=str(e)
            return response
    rclpy.init();node=Recorder()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
