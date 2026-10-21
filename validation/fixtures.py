"""Analytic synthetic room; never presented as a vehicle map or sensor recording."""
from pathlib import Path
import math
import numpy as np
from PIL import Image
import yaml
from shaoshao_hw.geometry import transform,matrix
from shaoshao_hw.index import IndexWriter,save_pcd
from shaoshao_hw.config import DEFAULTS

BOXES=[(-1.8,-1.2,1.1,1.6),(2.1,2.7,-1.6,-.7),(3.1,3.5,1.5,2.8)]

def make_scene():
    points=[]
    for x in np.arange(-4,5.01,.12):
        for y in np.arange(-3,4.01,.12):points.append((x,y,-.28288))
    for z in np.arange(-.2,1.31,.09):
        for x in np.arange(-4,5.01,.09):points.extend([(x,-3,z),(x,4,z)])
        for y in np.arange(-3,4.01,.09):points.extend([(-4,y,z),(5,y,z)])
        for x0,x1,y0,y1 in BOXES:
            for x in np.arange(x0,x1+.01,.08):points.extend([(x,y0,z),(x,y1,z)])
            for y in np.arange(y0,y1+.01,.08):points.extend([(x0,y,z),(x1,y,z)])
    return np.array(points,dtype=float)

def sensor_pose(x,y,yaw=0.):
    base=matrix([x,y,0,0,0,math.sin(yaw/2),math.cos(yaw/2)])
    base[:3,3]+=base[:3,:3]@np.array([-.011,-.02329,.04412])
    return base

def make_bundle(directory):
    root=Path(directory);root.mkdir(parents=True,exist_ok=True)
    points=make_scene();save_pcd(root/'site.pcd',points)
    settings=DEFAULTS['index'];writer=IndexWriter(root/'keyframes',settings)
    for x,y in [(-1,-1),(-.5,0),(0,0),(.5,.5),(1,0),(1,1),(0,2),(2,2)]:
        sensor=sensor_pose(x,y)
        writer.add(transform(points,np.linalg.inv(sensor)),sensor,len(writer.frames)+1)
    writer.finalize(root/'site.pcd')
    resolution=.025;x0,y0=-4.5,-3.5;width,height=400,320
    image=np.full((height,width),205,np.uint8)
    for iy in range(height):
        y=y0+(iy+.5)*resolution
        for ix in range(width):
            x=x0+(ix+.5)*resolution
            if -4<x<5 and -3<y<4:image[iy,ix]=254
            if ((-4.03<=x<=5.03 and (abs(y+3)<.03 or abs(y-4)<.03)) or
                (-3.03<=y<=4.03 and (abs(x+4)<.03 or abs(x-5)<.03)) or
                any(a<=x<=b and c<=y<=d for a,b,c,d in BOXES)):
                image[iy,ix]=0
    Image.fromarray(np.flipud(image)).save(root/'site.png')
    (root/'site.yaml').write_text(yaml.safe_dump(dict(image='site.png',resolution=resolution,
        origin=[x0,y0,0.],negate=0,occupied_thresh=.65,free_thresh=.196,mode='trinary')))
    return points

def ray_endpoints(sensor_map):
    angle=np.arange(720)*math.pi/360-math.pi
    local=np.column_stack([np.cos(angle),np.sin(angle)])
    direction=local@sensor_map[:2,:2].T;origin=sensor_map[:2,3]
    distances=np.full(720,15.)
    for axis,bound in [(0,-4),(0,5),(1,-3),(1,4)]:
        den=direction[:,axis]
        t=np.divide(bound-origin[axis],den,out=np.full(720,np.inf),where=np.abs(den)>1e-9)
        distances=np.minimum(distances,np.where(t>0,t,np.inf))
    for x0,x1,y0,y1 in BOXES:
        low=np.array([x0,y0]);high=np.array([x1,y1])
        inv=np.divide(1.,direction,out=np.full_like(direction,np.inf),where=np.abs(direction)>1e-9)
        a=(low-origin)*inv;b=(high-origin)*inv
        enter=np.maximum(np.minimum(a,b)[:,0],np.minimum(a,b)[:,1])
        leave=np.minimum(np.maximum(a,b)[:,0],np.maximum(a,b)[:,1])
        valid=(leave>=enter)&(enter>0)
        distances=np.minimum(distances,np.where(valid,enter,np.inf))
    return np.column_stack([local*distances[:,None],np.full(720,.2)])
