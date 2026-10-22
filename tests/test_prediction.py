from pathlib import Path
import math
import sys
import threading
import unittest
from types import SimpleNamespace as NS
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'web_panel'))
from shaoshao_hw.chassis import Feedback, FeedbackGate
from navcar_navigation import NavigationBridge


class FeedbackTests(unittest.TestCase):
    def sample(self, stamp=1., seq=1, epoch=1, allowed=True, omega=4.):
        return Feedback(stamp, seq, epoch, allowed, .1, .2, omega, .4)

    def test_actual_feedback_freshness_and_independent_spin(self):
        gate = FeedbackGate()
        self.assertFalse(gate.ready(1.))
        gate.update(self.sample(), 1.)
        self.assertTrue(gate.ready(1.1))
        self.assertEqual(gate.sample.omega, 4.)
        self.assertEqual((gate.sample.vx, gate.sample.vy), (.1, .2))
        self.assertFalse(gate.ready(1.151))

    def test_replay_does_not_renew_authority_or_timestamp(self):
        gate = FeedbackGate()
        gate.update(self.sample(), 1.)
        self.assertFalse(gate.update(self.sample(stamp=1.1), 1.1))
        self.assertFalse(gate.ready(1.16))
        self.assertTrue(gate.update(self.sample(stamp=1.2, seq=2, allowed=False), 1.2))
        self.assertFalse(gate.ready(1.2))
        self.assertTrue(gate.update(self.sample(stamp=1.3, epoch=2, seq=0), 1.3))
        self.assertTrue(gate.ready(1.3))
        gate.update(self.sample(stamp=1.4, epoch=1, seq=100), 1.4)
        self.assertEqual(gate.sample.authority_epoch, 2)

    def test_invalid_feedback_blocks_until_valid_measurement(self):
        gate = FeedbackGate()
        gate.update(self.sample(), 1.)
        with self.assertRaises(ValueError):
            gate.update(self.sample(stamp=1.1, seq=2, omega=float('nan')), 1.1)
        self.assertFalse(gate.ready(1.1))
        gate.update(self.sample(stamp=1.12, seq=3), 1.12)
        self.assertTrue(gate.ready(1.12))


class BridgeFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.bridge = NavigationBridge.__new__(NavigationBridge)
        self.bridge.chassis_gate = FeedbackGate()
        self.now = 1.
        self.bridge.node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=int(self.now*1e9))))
        self.cancels = 0
        def cancel():
            self.cancels += 1
        self.bridge.panel = NS(lock=threading.RLock(), commands={'auto': (1., (.1, .2, 0.))},
                               nav_status='active', cancel_navigation=cancel)
        self.bridge.auto_resume = {'x': 1.}
        self.bridge.resume_phase = 1
        self.bridge.motion = NS(stop=lambda: None)

    def deliver(self, seq=1, epoch=1, allowed=True, omega=4.):
        stamp = NS(sec=int(self.now), nanosec=round((self.now-int(self.now))*1e9))
        msg = NS(header=NS(frame_id='navcar_center', stamp=stamp), seq=seq, authority_epoch=epoch,
                 control_allowed=allowed, vx_mps=.1, vy_mps=.2, chassis_omega_radps=omega,
                 gimbal_relative_yaw_rad=.3)
        self.bridge.chassis_feedback(msg)

    def test_independent_chassis_spin_keeps_translation_and_goal(self):
        self.deliver()
        self.assertEqual(self.cancels, 0)
        self.assertIn('auto', self.bridge.panel.commands)

    def test_hardware_takeover_cancels_goal_and_deferred_resume(self):
        self.deliver()
        self.now = 1.02
        self.deliver(seq=0, epoch=2, allowed=False)
        self.assertEqual(self.cancels, 1)
        self.assertIsNone(self.bridge.auto_resume)
        self.assertEqual(self.bridge.panel.commands, {})


if __name__ == '__main__':
    unittest.main()
