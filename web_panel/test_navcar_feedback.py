import json
import math
import struct
import unittest

from navcar_feedback import ExtendedFeedback, measured_state, rotate_world_xy_to_frame
from navcar_move_web import Controller
from navcar_protocol import ACK, STATE, EXTENDED_STATE, Parser, Session, frame, stop_confirmed
from navcar_bridge import targets


def raw(values=(.2, .1, .3, 17), command_seq=8):
    return frame(STATE, command_seq, struct.pack('<4f', *values))


def extended(seq=1, tick=1000, epoch=3, flags=15, values=(.2, .1, .3, 0., 17.), command_seq=8, revision=1):
    return frame(EXTENDED_STATE, command_seq, struct.pack('<BBHIII5f',
        revision, flags, 0, seq, tick, epoch, *values))


def decoded(**kwargs):
    return Parser().feed(extended(**kwargs), 1)[0].values


class FeedbackTests(unittest.TestCase):
    def control(self):
        return Controller(dict(state_semantics='wheel_velocity_v2',
            feedback_protocol='experimental_0x82',
            require_extended_feedback_for_navigation=True,
            forward_units_per_mps=1., left_units_per_mps=1., rot_units_per_radps=1.,
            max_translation_units=.3, max_rot_units=6., yaw_target_deg=0.), clock=lambda: self.now)

    def setUp(self):
        self.now = 10.

    def test_v2_measurements_are_not_scaled_or_confused_with_targets(self):
        c=self.control();c.receive_serial(raw(), self.now)
        s=c.state();self.assertIsNone(s['feedback_targets'])
        self.assertAlmostEqual(s['feedback_measurements']['forward_mps'], .2)
        self.assertAlmostEqual(s['feedback_measurements']['left_mps'], -.1)
        self.assertAlmostEqual(s['feedback_measurements']['omega_radps'], .3)
        self.assertEqual(targets((.1,.2,.3),c.config), (.1,-.2,.3,0.))
        c.set_yaw(dict(angle_deg=45,client='test',seq=1))
        c.receive_serial(raw((0,0,0,12)),self.now)
        self.assertEqual(c.yaw,45)  # Measured yaw never overwrites the target.

    def test_invalid_wheels_are_fresh_link_but_not_zero_measurements(self):
        c=self.control();c.receive_serial(raw(),self.now)
        c.set_spin(dict(enabled=True,rate=2.,client='test',seq=1))
        self.now+=.01;c.receive_serial(raw((math.nan,math.nan,math.nan,21)),self.now)
        s=c.state();self.assertTrue(s['connected']);self.assertFalse(s['spin']['enabled'])
        self.assertFalse(s['feedback_measurements']['velocity_valid'])
        self.assertIsNone(s['feedback_measurements']['forward_mps'])
        json.dumps(s,allow_nan=False)
        with self.assertRaises(RuntimeError):c.command(dict(action='forward',strength=.1,client='test',seq=2))
        c.receive_serial(raw(),self.now+.01)
        self.assertFalse(c.spin_enabled)  # Recovery cannot resume spin.

    def test_xy_measurement_does_not_depend_on_rot_or_imu_yaw(self):
        c=Controller(dict(state_semantics='wheel_velocity_v2',feedback_protocol='navcar_uart1_v2',
            forward_units_per_mps=1.,left_units_per_mps=1.,rot_units_per_radps=1.,
            max_translation_units=.3,max_rot_units=6.,yaw_target_deg=0.),clock=lambda:self.now)
        c.receive_serial(raw((.12,-.04,math.nan,math.nan)),self.now)
        state=c.state();measure=state['feedback_measurements']
        self.assertTrue(measure['xy_valid']);self.assertTrue(state['connected'])
        self.assertAlmostEqual(measure['forward_mps'],.12);self.assertAlmostEqual(measure['left_mps'],.04)
        self.assertFalse(measure['omega_valid']);self.assertIsNone(measure['yaw_actual_deg'])
        self.assertEqual(state['feedback_error'],'')
        json.dumps(state,allow_nan=False)

    def test_world_xy_transform_uses_current_localization_heading(self):
        x,y=rotate_world_xy_to_frame(0.,1.,math.pi/2)
        self.assertAlmostEqual(x,1.)
        self.assertAlmostEqual(y,0.)
        self.assertEqual(rotate_world_xy_to_frame(.2,-.1,0.),(.2,-.1))

    def test_map_feedback_frame_configuration_is_explicit_and_validated(self):
        from navcar_bridge import load_config
        import yaml,tempfile
        from pathlib import Path
        value=dict(transport='navcar_uart1_v1',serial_port='/dev/test',baud=2000000,feedback_frame='map')
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'serial.yaml';p.write_text(yaml.safe_dump(value))
            self.assertEqual(load_config(p)['feedback_frame'],'map')
            value['feedback_frame']='world-ish';p.write_text(yaml.safe_dump(value))
            with self.assertRaisesRegex(ValueError,'feedback_frame'):load_config(p)

    def test_map_xy_v2_enables_navigation_without_yaw_or_omega_measurements(self):
        c=Controller(dict(state_semantics='wheel_velocity_v2',feedback_protocol='navcar_uart1_v2',
            feedback_frame='map',forward_units_per_mps=1.,left_units_per_mps=1.,rot_units_per_radps=1.,
            max_translation_units=.3,max_rot_units=6.,yaw_target_deg=0.),clock=lambda:self.now)
        c.receive_serial(raw((.2,-.1,math.nan,math.nan)),self.now)
        self.assertTrue(c.navigation_feedback_ready())
        self.assertEqual(c.yaw,0.)
        lease=c.acquire_navigation()
        c.navigation_command(dict(lease=lease,seq=1,forward=.08,left=.02))
        self.assertEqual(c.desired(),(.08,-.02,0.,0.))
        self.now+=.151
        self.assertFalse(c.navigation_feedback_ready())
        self.assertIn('超时',c.navigation_feedback_reason())

    def test_split_crc_and_extended_ack_demultiplexing(self):
        packet=extended();p=Parser()
        self.assertEqual(p.feed(packet[:11],0),[])
        self.assertEqual(p.feed(packet[11:],.01)[0].kind,EXTENDED_STATE)
        broken=bytearray(packet);broken[-1]^=1
        self.assertEqual(Parser().feed(broken,0),[])
        s=Session(measured=True);s.receive(raw(command_seq=7),0);s.poll((0,0,0,0),0)
        self.assertEqual(len(s.receive(packet,.01)),1)
        self.assertIsNotNone(s.pending)  # 0x82 flags/revision cannot be an ACK.
        s.receive(frame(ACK,8,b'\0'),.02);self.assertIsNone(s.pending)

    def test_rotating_body_velocity_into_lidar_axes(self):
        f=ExtendedFeedback()
        f.receive(decoded(values=(1.,0.,2.,math.pi/2,0)),10)
        self.assertAlmostEqual(f.sample['vx_mps'],0.,places=6)
        self.assertAlmostEqual(f.sample['vy_mps'],-1.,places=6)
        self.assertAlmostEqual(f.sample['chassis_omega_radps'],2.)
        self.assertTrue(f.ready(10.1));self.assertFalse(f.ready(10.151))

    def test_yaw_frame_feedback_needs_no_encoder_angle_or_encoder_validity_bit(self):
        f=ExtendedFeedback()
        f.receive(decoded(revision=2,flags=11,values=(.2,-.1,6.,0.,17.)),10.)
        self.assertTrue(f.ready(10.))
        self.assertAlmostEqual(f.sample['vx_mps'],.2)
        self.assertAlmostEqual(f.sample['vy_mps'],.1)
        self.assertEqual(f.sample['chassis_omega_radps'],6.)
        self.assertFalse(f.sample['gimbal_angle_available'])
        self.assertEqual(f.sample['source_velocity_frame'],'navcar_center')
        f.receive(decoded(revision=2,flags=11,seq=2,tick=1010,values=(.2,-.1,-6.,0.,-175.)),10.01)
        # Chassis spin and absolute IMU yaw cannot rotate an already converted velocity again.
        self.assertAlmostEqual(f.sample['vx_mps'],.2)
        self.assertAlmostEqual(f.sample['vy_mps'],.1)

    def test_yaw_frame_feedback_keeps_wheel_and_authority_gates(self):
        for flags,values in [(9,(0,0,0,0,17)),(11,(math.nan,0,0,0,17)),(10,(0,0,0,0,17))]:
            f=ExtendedFeedback();f.receive(decoded(revision=2,flags=flags,values=values),10.)
            self.assertFalse(f.ready(10.))
        f=ExtendedFeedback();f.receive(decoded(revision=2,flags=11),10.)
        f.receive(decoded(revision=2,flags=10,epoch=4,seq=2,tick=1010),10.01)
        self.assertFalse(f.ready(10.01));self.assertFalse(f.error)

    def test_velocity_frame_cannot_change_in_a_live_session(self):
        f=ExtendedFeedback();f.receive(decoded(revision=2,flags=11),10.)
        f.receive(decoded(seq=2,tick=1010),10.01)
        self.assertIn('坐标系',f.error)
        self.assertFalse(f.ready(10.01))

    def test_command_sequence_repeats_but_measurement_sequence_advances(self):
        f=ExtendedFeedback();f.receive(decoded(seq=1),10)
        self.assertTrue(f.receive(decoded(seq=2,tick=1010),10.01))
        self.assertEqual(f.sample['seq'],2)
        stamp=f.sample['stamp']
        self.assertFalse(f.receive(decoded(seq=2,tick=1010),10.2))
        self.assertEqual(f.sample['stamp'],stamp);self.assertFalse(f.ready(10.2))

    def test_counter_wrap_is_valid_but_restart_and_replay_latch_fault(self):
        f=ExtendedFeedback();f.receive(decoded(seq=0xffffffff,tick=0xfffffffa),10)
        self.assertTrue(f.receive(decoded(seq=0,tick=4),10.01))
        self.assertEqual(f.sample['seq'],2**32)
        self.assertTrue(f.ready(10.01))
        f.receive(decoded(seq=0,tick=0),10.02)
        self.assertTrue(f.error);self.assertFalse(f.ready(10.02))
        self.assertFalse(f.receive(decoded(seq=1,tick=14),10.03))

    def test_queued_measurements_keep_their_age(self):
        f=ExtendedFeedback();f.receive(decoded(),10)
        f.receive(decoded(seq=2,tick=1010),10.3)
        self.assertAlmostEqual(f.sample['stamp'],10.01)
        self.assertFalse(f.ready(10.3))

    def test_takeover_invalidity_and_authority_epochs(self):
        f=ExtendedFeedback();f.receive(decoded(),10);epoch=f.sample['authority_epoch']
        f.receive(decoded(seq=2,tick=1010,epoch=4,flags=14),10.01)
        self.assertFalse(f.ready(10.01));self.assertEqual(f.sample['authority_epoch'],epoch+1)
        f.receive(decoded(seq=3,tick=1020,epoch=5),10.02);self.assertTrue(f.ready(10.02))
        f.receive(decoded(seq=4,tick=1030,epoch=5,flags=14),10.03)
        self.assertTrue(f.error)  # Permission toggles must increment epoch.
        for values,flags in [((math.nan,0,0,0,0),15),((0,0,0,math.nan,0),15),((0,0,0,0,0),11)]:
            f=ExtendedFeedback();f.receive(decoded(values=values,flags=flags),10)
            self.assertFalse(f.ready(10));self.assertFalse(f.sample['control_allowed'])
            json.dumps(f.sample,allow_nan=False)

    def test_navigation_requires_extended_feedback_and_revokes_on_takeover(self):
        c=self.control();c.receive_serial(raw(),10)
        with self.assertRaisesRegex(RuntimeError,'0x82'):c.acquire_navigation()
        c.receive_serial(extended(),10);lease=c.acquire_navigation()
        c.navigation_command(dict(lease=lease,seq=1,forward=.05,left=.02))
        self.assertEqual(c.desired()[:2],(.05,-.02))
        self.now=10.01;c.receive_serial(extended(seq=2,tick=1010,epoch=4,flags=14),self.now)
        self.assertIsNone(c.navigation_lease);self.assertEqual(c.desired()[:3],(0,0,0))
        with self.assertRaises(RuntimeError):c.navigation_command(dict(lease=lease,seq=2,forward=.05,left=0))
        self.now=10.02;c.receive_serial(extended(seq=3,tick=1020,epoch=5),self.now)
        self.assertIsNone(c.navigation_lease)

    def test_zero_ack_requires_measured_stop_with_tolerance(self):
        s=Session(measured=True);s.receive(raw(command_seq=7),0)
        s.poll((0,0,0,17),.01);s.receive(frame(ACK,8,b'\0'),.02)
        s.receive(raw(values=(.2,0,0,11),command_seq=8),.03)
        self.assertFalse(stop_confirmed(s,.01))
        s.receive(raw(values=(.01,-.01,.02,11),command_seq=8),.04)
        self.assertTrue(stop_confirmed(s,.01))
        s.receive(raw(values=(math.nan,0,0,11),command_seq=8),.05)
        self.assertFalse(stop_confirmed(s,.01))

    def test_zero_stop_duplicate_ack_is_valid_after_first_ack_was_lost(self):
        s=Session(measured=True);s.receive(raw(command_seq=7),0)
        command=s.poll((0,0,0,17),.01)
        self.assertEqual(s.poll((0,0,0,17),.061),command)
        s.receive(frame(ACK,8,b'\1'),.07)
        s.receive(raw(values=(.01,0,0,12),command_seq=8),.08)
        self.assertTrue(stop_confirmed(s,.01))

    def test_official_v2_uses_body_measurements_without_waiting_for_draft_extension(self):
        c=self.control();c.config['feedback_protocol']='navcar_uart1_v2'
        c=Controller(c.config,clock=lambda:self.now)
        self.assertIn('0x81',c.state()['navigation_feedback_reason'])
        c.receive_serial(raw(),self.now)
        s=c.state()
        self.assertTrue(s['connected']);self.assertEqual(s['feedback_error'],'')
        self.assertEqual(s['feedback_protocol'],'navcar_uart1_v2')
        self.assertNotIn('0x82',s['navigation_feedback_reason'])
        self.assertFalse(s['feedback_capabilities']['control_mode_reported'])
        with self.assertRaisesRegex(RuntimeError,'朝向转换和 UART1 控制模式'):c.acquire_navigation()
        c.receive_serial(extended(),self.now)
        self.assertIsNone(c.extended.sample)  # Unrequested draft cannot change official semantics.
        c.command(dict(action='forward',strength=.1,client='test',seq=1))
        self.assertEqual(c.desired(),(.1,0.,0.,17.))

    def test_uart_not_connected_is_not_reported_as_missing_navigation_capability(self):
        c=Controller(dict(state_semantics='wheel_velocity_v2',feedback_protocol='navcar_uart1_v2',
            forward_units_per_mps=1.,left_units_per_mps=1.,rot_units_per_radps=1.,
            max_translation_units=.3,max_rot_units=6.,yaw_target_deg=0.),clock=lambda:self.now)
        self.assertIn('等待下位机 V2',c.navigation_feedback_reason())
        c.receive_serial(raw(),self.now)
        self.assertIn('朝向转换',c.navigation_feedback_reason())
        self.now+=.301
        self.assertIn('XY 速度反馈超时',c.navigation_feedback_reason())


if __name__=='__main__':unittest.main()
