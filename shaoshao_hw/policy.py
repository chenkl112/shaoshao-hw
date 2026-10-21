import math
import numpy as np
from .geometry import delta,interpolate

def quality_ok(metrics,p):
    fields=('error','overlap','min_eigenvalue','condition')
    if not metrics.get('converged') or not all(math.isfinite(metrics.get(k,float('nan'))) for k in fields):return False
    return (0<=metrics['error']<=p['max_error'] and p['min_overlap']<=metrics['overlap']<=1 and
            metrics['min_eigenvalue']>=p['min_eigenvalue'] and
            1<=metrics['condition']<=p['max_condition'])

def unique_best(candidates,p):
    good=sorted((x for x in candidates if quality_ok(x[1],p)),key=lambda x:x[1]['error'])
    if not good:return None
    best=good[0]
    for other in good[1:]:
        d,a=delta(best[0],other[0])
        if (d>=.5 or a>=.25) and other[1]['error']<max(best[1]['error']*1.25,best[1]['error']+1e-6):
            return None
    return best

class Policy:
    def __init__(self,p):
        self.p=p;self.correction=None;self.raw=None;self.pending=None;self.confirmations=0
        self.initialized=False;self.last_stamp=None;self.accepted_at=None;self.rejected_at=None
        self.accepted=0;self.rejected=0;self.reason='等待自动全局定位';self.metrics={}
        self.odom=None;self.odom_at=None;self.distance=0.;self.accepted_distance=0.
        self.scan_at=None;self.clearance=0.;self.scan_valid=False;self.fault=False
    def odometry(self,m,stamp):
        if self.odom_at is not None:
            if stamp<=self.odom_at:return
            d,_=delta(self.odom,m)
            if d>max(.15,1.5*(stamp-self.odom_at)):
                self.fault=True;self.lose('里程计跳变，正在重新全局定位')
            self.distance+=d
        self.odom=m;self.odom_at=stamp
    def scan(self,stamp,points):
        if self.scan_at is not None and stamp<=self.scan_at:return
        self.scan_at=stamp
        sectors={int((math.atan2(y,x)+math.pi)/(math.pi/4))%8 for x,y in points}
        self.scan_valid=len(points)>=60 and len(sectors)>=6
        self.clearance=min((math.hypot(x,y) for x,y in points),default=0.)
    def reject(self,why,now):
        self.rejected+=1;self.rejected_at=now;self.reason=why;self.confirmations=0
        if not self.initialized:self.pending=None
    def lose(self,why):
        self.initialized=False;self.pending=None;self.confirmations=0;self.reason=why
    def observe(self,m,metrics,stamp,now,global_match=False):
        if self.last_stamp is not None and stamp<=self.last_stamp:return False
        self.last_stamp=stamp
        if now-stamp>.5 or stamp>now+.05:
            self.reject('配准结果过期',now);return False
        self.metrics={k:(v if isinstance(v,bool) else float(v) if isinstance(v,(int,float)) else v) for k,v in metrics.items()}
        if not quality_ok(metrics,self.p) or not np.isfinite(m).all():
            self.reject('点云匹配质量不足',now);return False
        ref=self.raw if self.initialized else self.pending
        if ref is not None:
            d,a=delta(ref,m)
            dm,da=(self.p['max_step_m'],self.p['max_step_rad']) if self.initialized else (.1,.1)
            if d>dm or a>da:
                self.reject('校正跳变超限',now);return False
        self.confirmations+=1
        if not self.initialized:
            if not global_match:
                self.reject('等待唯一全局候选',now);return False
            self.pending=m.copy()
            if self.confirmations<self.p['confirmations']:
                self.reason=f'正在确认全局定位（{self.confirmations}/{self.p["confirmations"]}）';return False
            self.correction=m.copy();self.initialized=True;self.fault=False
        else:self.correction=interpolate(self.correction,m,self.p['alpha'])
        self.raw=m.copy();self.accepted_at=stamp;self.accepted_distance=self.distance;self.accepted+=1
        self.reason='地图定位就绪'
        return True
    def status(self,now,radius,padding,max_speed):
        age=None if self.accepted_at is None else max(0.,now-self.accepted_at)
        travelled=max(0.,self.distance-self.accepted_distance)
        result={'ready':False,'state':'acquiring','message':self.reason,'speed_limit_mps':0.,
                'last_correction_age_s':age,'distance_since_correction_m':travelled,
                'accepted':self.accepted,'rejected':self.rejected,'confirmations':self.confirmations,
                'metrics':self.metrics}
        if not self.initialized:return result
        why=None
        if self.odom_at is None or now-self.odom_at>1:why='里程计更新超时'
        if self.scan_at is None or now-self.scan_at>self.p['scan_timeout_s'] or not self.scan_valid:
            why=why or '扫描超时或回波覆盖不足'
        if age>self.p['timeout_s']:why=why or '有效地图校正超时'
        if why:
            self.lose(why);result.update(state='lost',message=why);return result
        degraded=(age>self.p['fresh_s'] or (self.rejected_at is not None and self.rejected_at>self.accepted_at) or
                  self.confirmations<self.p['confirmations'])
        if degraded:
            if travelled>self.p['hold_distance_m']:
                self.lose('退化累计移动超限');result.update(state='lost',message=self.reason);return result
            # Registration quality is not a metric covariance. Reserve the entire
            # configured hold budget, plus actual travel and scan latency, rather
            # than inventing an accuracy from the GICP fitness number.
            required=radius+padding+self.p['hold_distance_m']+travelled+max_speed*max(0.,now-self.scan_at)
            result.update(state='degraded',required_clearance_m=required,observed_clearance_m=self.clearance)
            if self.clearance<=required:
                result['message']='定位退化且近场余量不足，已停车';return result
            result.update(ready=True,message='定位退化，低速衔接',speed_limit_mps=min(max_speed,self.p['degraded_speed_mps']))
            return result
        result.update(ready=True,state='tracking',message='地图定位就绪',speed_limit_mps=max_speed)
        return result
