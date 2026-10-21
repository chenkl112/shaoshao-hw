import numpy as np

def decode(msg):
    # XYZ only; respect point/row strides and endian flag.
    fields={f.name:f for f in msg.fields}
    if not all(k in fields and fields[k].datatype in (7,8) for k in ('x','y','z')):
        raise ValueError('XYZ floating fields required')
    endian='>' if msg.is_bigendian else '<'
    arrays=[]
    for name in ('x','y','z'):
        f=fields[name];dt=np.dtype(endian+('f4' if f.datatype==7 else 'f8'))
        arrays.append(np.ndarray((msg.height,msg.width),dt,buffer=msg.data,
            offset=f.offset,strides=(msg.row_step,msg.point_step)).reshape(-1))
    p=np.column_stack(arrays)
    return p[np.isfinite(p).all(axis=1)]

def encode(points,frame,stamp):
    from sensor_msgs.msg import PointCloud2,PointField
    p=np.asarray(points,dtype='<f4').reshape(-1,3)
    msg=PointCloud2();msg.header.frame_id=frame;msg.header.stamp=stamp
    msg.height=1;msg.width=len(p);msg.is_bigendian=False;msg.is_dense=True
    msg.point_step=12;msg.row_step=12*len(p);msg.data=p.tobytes()
    msg.fields=[PointField(name=n,offset=4*i,datatype=7,count=1) for i,n in enumerate(('x','y','z'))]
    return msg
