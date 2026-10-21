import math
import numpy as np

def clip(vector,limit):
    vector=np.asarray(vector,float);length=float(np.linalg.norm(vector))
    return vector*(limit/length) if length>limit and length else vector

class MotionLimiter:
    def __init__(self,settings):
        self.p=settings;self.velocity=np.zeros(2);self.acceleration=np.zeros(2)
    def stop(self):
        self.velocity[:]=0.;self.acceleration[:]=0.
        return 0.,0.
    def step(self,target,dt,limit,emergency=False):
        if emergency or not np.isfinite(target).all() or not 0<dt<=.1 or limit<=0:return self.stop()
        # Leave a discrete integration guard so normal endpoint settling does not
        # repeatedly hit the hard speed-envelope projection.
        target=clip(target,max(0.,limit-4*self.p['jerk_mps3']*dt*dt))
        # Per-axis allocation guarantees the vector acceleration/jerk bounds.
        j=self.p['jerk_mps3']/math.sqrt(2)
        error=target-self.velocity
        amax=min(self.p['acceleration_mps2'],self.p['deceleration_mps2'])/math.sqrt(2)
        desired=np.sign(error)*np.minimum(amax,np.sqrt(2*j*np.abs(error)))
        change=np.clip(desired-self.acceleration,-j*dt,j*dt)
        self.acceleration+=change
        finish=error/dt
        can_finish=(np.abs(finish-(self.acceleration-change))<=j*dt)&(np.abs(finish)<=j*dt)
        self.acceleration=np.where(can_finish,finish,self.acceleration)
        self.velocity+=self.acceleration*dt
        # Speed envelope and degraded limits are safety bounds, not comfort goals.
        if np.linalg.norm(self.velocity)>limit:
            self.velocity=clip(self.velocity,limit);self.acceleration[:]=0.
        return tuple(float(x) for x in self.velocity)

class CollisionGrid:
    def __init__(self,values,resolution,origin,stamp):
        from scipy.ndimage import distance_transform_edt
        values=np.asarray(values)
        if values.ndim!=2 or not values.size or not .01<=resolution<=.1 or not np.isfinite(origin).all():
            raise ValueError('Invalid local collision grid')
        self.resolution=resolution;self.origin=origin;self.stamp=stamp
        self.clearance=distance_transform_edt((values>=0)&(values<100))*resolution
        self.height,self.width=values.shape
    def safe(self,position,velocity,radius,now,acceleration,jerk):
        if now-self.stamp>.5:return False
        x,y,yaw=position;vx,vy=velocity
        speed=math.hypot(vx,vy)
        # Account for the acceleration ramp before constant braking can be reached.
        stop_time=speed/acceleration+acceleration/jerk+.2
        # A small configured deceleration must never silently shorten the guard.
        if stop_time>3.:return False
        horizon=max(.2,stop_time)
        dt=min(.025,self.resolution/(2*max(speed,.01)))
        c,s=math.cos(yaw),math.sin(yaw)
        wx,wy=c*vx-s*vy,s*vx+c*vy
        ox,oy,oyaw=self.origin;co,so=math.cos(oyaw),math.sin(oyaw)
        for t in np.arange(0,horizon+dt,dt):
            # Constant initial velocity is conservative during straight braking.
            px,py=x+wx*t-ox,y+wy*t-oy
            ix=math.floor((co*px+so*py)/self.resolution)
            iy=math.floor((-so*px+co*py)/self.resolution)
            if not 0<=ix<self.width or not 0<=iy<self.height:return False
            if self.clearance[iy,ix]<radius+self.resolution*math.sqrt(2)/2:return False
        return True
