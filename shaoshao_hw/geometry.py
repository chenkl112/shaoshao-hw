import math
import numpy as np

def wrap(x):return math.atan2(math.sin(x),math.cos(x))

def matrix(pose):
    x,y,z,qx,qy,qz,qw=pose
    q=np.array([qx,qy,qz,qw],float)
    if not np.isfinite(pose).all() or abs(np.linalg.norm(q)-1)>.01:raise ValueError('Invalid pose')
    q/=np.linalg.norm(q);xq,yq,zq,w=q
    m=np.eye(4)
    m[:3,:3]=[[1-2*(yq*yq+zq*zq),2*(xq*yq-zq*w),2*(xq*zq+yq*w)],
               [2*(xq*yq+zq*w),1-2*(xq*xq+zq*zq),2*(yq*zq-xq*w)],
               [2*(xq*zq-yq*w),2*(yq*zq+xq*w),1-2*(xq*xq+yq*yq)]]
    m[:3,3]=[x,y,z]
    return m

def pose(m):
    from scipy.spatial.transform import Rotation
    return [*m[:3,3].tolist(),*Rotation.from_matrix(m[:3,:3]).as_quat().tolist()]

def transform(points,m):return np.asarray(points)@m[:3,:3].T+m[:3,3]

def delta(a,b):
    rotation=a[:3,:3].T@b[:3,:3]
    angle=math.acos(float(np.clip((np.trace(rotation)-1)/2,-1,1)))
    return float(np.linalg.norm(a[:3,3]-b[:3,3])),angle

def interpolate(a,b,alpha):
    from scipy.spatial.transform import Rotation,Slerp
    out=np.eye(4);out[:3,3]=a[:3,3]+alpha*(b[:3,3]-a[:3,3])
    out[:3,:3]=Slerp([0.,1.],Rotation.from_matrix(np.stack([a[:3,:3],b[:3,:3]])))(alpha).as_matrix()
    return out

def pose_msg(msg):return matrix([msg.position.x,msg.position.y,msg.position.z,
    msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w])

def write_pose(msg,m):
    p=pose(m);msg.position.x,msg.position.y,msg.position.z=p[:3]
    msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w=p[3:]
    return msg
