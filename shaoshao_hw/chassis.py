"""Transport-independent measured feedback gate; contains no wheel kinematics."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Feedback:
    stamp: float
    seq: int
    authority_epoch: int
    control_allowed: bool
    vx: float
    vy: float
    omega: float
    gimbal_yaw: float
    frame: str = 'navcar_center'


class FeedbackGate:
    def __init__(self, timeout=0.15):
        self.timeout = timeout
        self.sample = None
        self.invalid = False

    def update(self, sample, now):
        values = (sample.stamp, sample.vx, sample.vy, sample.omega, sample.gimbal_yaw)
        if (not all(math.isfinite(v) for v in values) or sample.frame != 'navcar_center'
                or type(sample.control_allowed) is not bool
                or any(type(v) is not int or not 0 <= v < 2**64 for v in (sample.seq, sample.authority_epoch))
                or not -0.05 <= now-sample.stamp <= self.timeout
                or math.hypot(sample.vx, sample.vy) > 3 or abs(sample.omega) > 30):
            self.invalid = True
            raise ValueError('Invalid or stale actual chassis feedback')
        old = self.sample
        if old is not None:
            if sample.authority_epoch < old.authority_epoch:
                return False
            if sample.authority_epoch == old.authority_epoch and (sample.seq <= old.seq or sample.stamp <= old.stamp):
                return False
        self.sample = sample
        self.invalid = False
        return old is not None and (sample.authority_epoch != old.authority_epoch
                                   or (old.control_allowed and not sample.control_allowed))

    def ready(self, now):
        return bool(not self.invalid and self.sample is not None and self.sample.control_allowed
                    and -0.05 <= now-self.sample.stamp <= self.timeout)

    def reason(self, now):
        if self.invalid:
            return '底盘实际速度反馈无效'
        if self.sample is None:
            return '等待新协议的底盘实际速度反馈'
        if not -0.05 <= now-self.sample.stamp <= self.timeout:
            return '底盘实际速度反馈超时'
        if not self.sample.control_allowed:
            return '拨杆未授权上位机控制'
        return ''
