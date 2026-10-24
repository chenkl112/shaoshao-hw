"""Measured UART 0x81 XY values and optional legacy angular fields."""
import math
import time


def measured_state(values, frame='navcar_chassis'):
    vy, vx, omega, yaw = values
    xy_valid = all(math.isfinite(v) for v in (vy, vx)) and math.hypot(vy, vx) <= 3
    omega_valid = math.isfinite(omega) and abs(omega) <= 30
    return {'frame': frame, 'velocity_valid': xy_valid, 'xy_valid': xy_valid,
            # The V2 payload calls these vy/vx. In the confirmed map-frame
            # configuration they map to map X/Y using the same ROS signs.
            'forward_mps': vy if xy_valid else None, 'left_mps': -vx if xy_valid else None,
            'omega_radps': omega if omega_valid else None, 'omega_valid': omega_valid,
            'yaw_actual_deg': yaw if math.isfinite(yaw) else None,
            'source': 'navcar_uart1_v2_0x81'}


def rotate_world_xy_to_frame(vx, vy, frame_yaw):
    """Rotate a world/map vector into a child frame with yaw in map."""
    if not all(math.isfinite(v) for v in (vx, vy, frame_yaw)):
        raise ValueError('Velocity and frame yaw must be finite')
    c, s = math.cos(frame_yaw), math.sin(frame_yaw)
    return c*vx+s*vy, -s*vx+c*vy


class ExtendedFeedback:
    """0x82 measured feedback with independent sample and takeover counters.

    Revision 2 supplies velocity already in lidar/gimbal axes; no encoder angle.
    Revision 1 remains supported for senders supplying body velocity and angle.

    Receipt-time anchoring bounds queue delay using the MCU monotonic sample tick.
    A reboot/backwards counter latches a fault; restart the serial service to resync.
    """
    def __init__(self):
        self.sample = None
        self.received = None
        self.error = ''
        self.last = None
        self.last_allowed = None
        self.revision = None
        self.offset = None
        self.ticks = self.sequence = 0
        self.epoch_base = time.time_ns()
        self.epoch = 0

    def receive(self, values, now):
        if self.error:
            return False
        revision, flags, reserved, seq, tick, epoch, vy, vx, omega, angle, yaw = values
        if (revision not in (1, 2) or reserved or flags & ~(15 if revision == 1 else 11)
                or (revision == 2 and angle != 0)):
            self.error = '底盘扩展反馈版本或标志无效，请重启控制服务'
            return False
        if self.revision is not None and revision != self.revision:
            self.error = '底盘速度坐标系发生变化，请重启控制服务重新同步'
            return False
        self.revision = revision
        if self.last is not None:
            old_seq, old_tick, old_epoch = self.last
            ds, dt, de = ((seq-old_seq) & 0xffffffff, (tick-old_tick) & 0xffffffff,
                          (epoch-old_epoch) & 0xffffffff)
            # Duplicate measurement is not a fresh heartbeat.
            if ds == 0 and dt == 0 and de == 0:
                return False
            if not 0 < ds < 0x80000000 or not 0 < dt < 0x80000000 or de >= 0x80000000:
                self.error = '底盘测量序号/时钟倒退，请重启控制服务'
                return False
            if bool(flags & 1) != self.last_allowed and de == 0:
                self.error = '底盘控制授权变化未更新 epoch，请检查下位机固件'
                return False
            self.sequence += ds
            self.ticks += dt
            self.epoch += de
        else:
            self.sequence, self.ticks = seq, tick
        self.last = seq, tick, epoch
        self.last_allowed = bool(flags & 1)
        self.offset = min(now-self.ticks*.001, self.offset) if self.offset is not None else now-self.ticks*.001
        stamp = self.offset+self.ticks*.001
        body = measured_state((vy, vx, omega, yaw))
        valid = bool(flags & 2 and body['velocity_valid'])
        if revision == 1:
            valid = valid and bool(flags & 4) and math.isfinite(angle) and abs(angle) <= math.pi+.001
        c, s = (math.cos(angle), math.sin(angle)) if valid and revision == 1 else (1., 0.)
        forward, left = (vy, -vx) if valid else (0., 0.)
        self.received = now
        self.sample = {'stamp': stamp, 'seq': self.sequence,
            'authority_epoch': self.epoch_base+self.epoch,
            'control_allowed': bool(flags & 1) and valid,
            'velocity_valid': valid, 'frame': 'navcar_center',
            'wire_revision': revision,
            'source_velocity_frame': 'navcar_center' if revision == 2 else 'navcar_chassis',
            'gimbal_angle_available': revision == 1 and valid,
            'vx_mps': c*forward+s*left, 'vy_mps': -s*forward+c*left,
            'chassis_omega_radps': omega if valid and body['omega_valid'] else 0.,
            # Legacy ROS field: reserved zero in revision 2, unused by control.
            'gimbal_relative_yaw_rad': angle if valid and revision == 1 else 0.,
            'yaw_actual_deg': yaw if flags & 8 and math.isfinite(yaw) else None}
        return True

    def ready(self, now):
        return bool(not self.error and self.sample and self.sample['control_allowed']
                    and -.05 <= now-self.sample['stamp'] <= .15)

    def reason(self, now):
        if self.error:
            return self.error
        if self.sample is None:
            return '等待 0x82 扩展反馈：云台坐标系实测速度与 UART1 控制授权'
        if not -.05 <= now-self.sample['stamp'] <= .15:
            return '底盘扩展速度反馈超时'
        if not self.sample['velocity_valid']:
            return '底盘实测速度或速度坐标转换无效'
        if not self.sample['control_allowed']:
            return '下位机未授权 UART1 控制（遥控器接管或电机未就绪）'
        return ''
