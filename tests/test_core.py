from pathlib import Path
import math
import sys
import tempfile
import unittest
from types import SimpleNamespace as NS
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'web_panel'))
from shaoshao_hw.config import load,DEFAULTS
from shaoshao_hw.geometry import matrix,delta,transform
from shaoshao_hw.index import IndexWriter,RetrievalIndex,descriptor,descriptor_distance,save_pcd
from shaoshao_hw.policy import Policy,quality_ok,unique_best
from shaoshao_hw.motion import MotionLimiter,CollisionGrid
from navcar_move_web import Controller,receive_ros_command
from navcar_protocol import STATE,frame
from navcar_navigation import build_navigation_config
import struct

def pose(x=0,y=0,yaw=0):return matrix([x,y,0,0,0,math.sin(yaw/2),math.cos(yaw/2)])
def metrics(error=.01):return dict(converged=True,error=error,overlap=.95,min_eigenvalue=2000.,condition=500.)
def observed(p,t,position=None,clearance=2.):
    p.odometry(pose() if position is None else position,t)
    p.scan(t,[(clearance*math.cos(i*math.pi/30),clearance*math.sin(i*math.pi/30)) for i in range(60)])
def acquired():
    p=Policy(DEFAULTS['registration'])
    for t in (1.,1.1,1.2):observed(p,t);p.observe(pose(1,2),metrics(),t,t,True)
    return p
def status(p,t):return p.status(t,.297658,.02,.5)

class CoreTests(unittest.TestCase):
    def test_three_independent_confirmations_and_no_replay_renewal(self):
        p=Policy(DEFAULTS['registration']);observed(p,1.)
        self.assertFalse(p.observe(pose(),metrics(),1.,1.,True))
        self.assertFalse(p.observe(pose(),metrics(),1.,1.1,True));self.assertEqual(p.confirmations,1)
        p.observe(pose(),metrics(),1.1,1.1,True);self.assertIsNone(p.correction)
        self.assertTrue(p.observe(pose(),metrics(),1.2,1.2,True))
        self.assertEqual(p.accepted_at,1.2)
        self.assertFalse(p.observe(pose(),metrics(),1.1,1.3,True));self.assertEqual(p.accepted_at,1.2)

    def test_quality_bad_information_and_ambiguous_global_candidates(self):
        for field,value in [('converged',False),('overlap',.2),('error',float('nan')),
                            ('min_eigenvalue',0),('condition',1e7)]:
            m=metrics();m[field]=value;self.assertFalse(quality_ok(m,DEFAULTS['registration']))
        self.assertIsNone(unique_best([(pose(),metrics(.01)),(pose(2),metrics(.011))],DEFAULTS['registration']))
        result=unique_best([(pose(),metrics(.01)),(pose(2),metrics(.02))],DEFAULTS['registration'])
        self.assertLess(delta(result[0],pose())[0],1e-9)

    def test_bad_correction_is_not_published_and_local_motion_does_not_change_transform(self):
        p=acquired();before=p.correction.copy()
        observed(p,1.3,pose(.04));self.assertTrue(p.observe(pose(1,2),metrics(),1.3,1.3))
        self.assertTrue(np.allclose(p.correction,before))
        self.assertFalse(p.observe(pose(1.5,2),metrics(),1.4,1.4))
        self.assertTrue(np.allclose(p.correction,before));self.assertEqual(status(p,1.4)['state'],'degraded')
        self.assertEqual(status(p,1.4)['speed_limit_mps'],.05)

    def test_hold_distance_timeout_and_scan_age_stop(self):
        p=acquired();p.reject('one bad update',1.3);observed(p,1.4,pose(.03))
        self.assertTrue(status(p,1.4)['ready'])
        observed(p,1.5,pose());self.assertFalse(status(p,1.5)['ready'])
        p=acquired();observed(p,3.3)
        self.assertEqual(status(p,3.3)['state'],'lost')
        p=acquired();p.odometry(pose(),1.8)
        self.assertFalse(status(p,1.8)['ready'])

    def test_degraded_cannot_bypass_narrow_passage_by_going_slowly(self):
        p=acquired();p.reject('bad',1.3);observed(p,1.3,clearance=.325)
        result=status(p,1.3)
        self.assertFalse(result['ready']);self.assertIn('余量不足',result['message'])

    def test_velocity_acceleration_jerk_and_zero_settling(self):
        limiter=MotionLimiter(DEFAULTS['motion']);values=[]
        for i in range(1000):
            target=(.3,.4) if i<300 else ((-.3,.4) if i<600 else (0,0))
            values.append(limiter.step(target,.02,.5))
        v=np.array(values);a=np.diff(v,axis=0)/.02;j=np.diff(a,axis=0)/.02
        self.assertLessEqual(np.linalg.norm(v,axis=1).max(),.5+1e-9)
        self.assertLessEqual(np.linalg.norm(a,axis=1).max(),.3+1e-8)
        self.assertLessEqual(np.linalg.norm(j,axis=1).max(),1.+1e-7)
        self.assertTrue(np.array_equal(v[-1],[0,0]))
        self.assertEqual(limiter.step((.5,0),.02,.5,emergency=True),(0,0))

    def test_emergency_and_lower_cap_bypass_comfort_delay(self):
        limiter=MotionLimiter(DEFAULTS['motion'])
        for _ in range(300):limiter.step((.4,0),.02,.5)
        self.assertLessEqual(math.hypot(*limiter.step((.4,0),.02,.05)),.05+1e-9)
        self.assertEqual(limiter.step((.5,0),.3,.5),(0,0))
        self.assertEqual(limiter.step((float('nan'),0),.02,.5),(0,0))

    def test_collision_guard_checks_unknown_staleness_and_full_radius(self):
        data=np.zeros((160,160),int);data[:,110]=100
        grid=CollisionGrid(data,.025,(-2,-2,0),1.)
        self.assertTrue(grid.safe((0,0,0),(.1,0),.317658,1.,.3,1.))
        self.assertFalse(grid.safe((0,0,0),(.5,0),.317658,1.,.3,1.))
        self.assertFalse(grid.safe((0,0,0),(.1,0),.317658,1.6,.3,1.))
        self.assertFalse(grid.safe((0,0,0),(.1,0),.317658,1.,.001,1.))
        data[:,85]=-1;grid=CollisionGrid(data,.025,(-2,-2,0),1.)
        self.assertFalse(grid.safe((0,0,0),(.1,0),.317658,1.,.3,1.))

    def test_descriptor_yaw_retrieval_and_hash_binding(self):
        rng=np.random.default_rng(7)
        points=rng.uniform([-4,-3,-.3],[5,4,1.2],(5000,3))
        target=pose(1,2,.2)
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);save_pcd(root/'site.pcd',transform(points,target))
            writer=IndexWriter(root/'keyframes',DEFAULTS['index'])
            self.assertTrue(writer.add(points,target,1));self.assertFalse(writer.add(points,target,2))
            writer.finalize(root/'site.pcd')
            index=RetrievalIndex(root/'keyframes',root/'site.pcd')
            rotation=pose(yaw=math.pi/3)
            query=transform(points,np.linalg.inv(rotation))
            candidates=index.candidates(query,np.eye(4))
            d,a=delta(candidates[0],target@rotation)
            self.assertLess(d,.01);self.assertLess(a,.11)
            with (root/'site.pcd').open('ab') as f:f.write(b'changed')
            with self.assertRaises(ValueError):RetrievalIndex(root/'keyframes',root/'site.pcd')

    def test_ros_command_age_lease_manual_takeover_and_multiplier(self):
        controller=Controller(dict(forward_units_per_mps=10,left_units_per_mps=10,rot_units_per_radps=1,
            max_translation_units=30,max_rot_units=6),clock=lambda:10.,navigation_max_speed_mps=.5)
        controller.session.receive(frame(STATE,1,struct.pack('<4f',0,0,0,17)),10.);controller.yaw=17
        lease=controller.acquire_navigation()
        msg=NS(header=NS(frame_id='navcar_center',stamp=NS(sec=10,nanosec=0)),lease=lease,seq=1,
            twist=NS(linear=NS(x=.3,y=.4,z=0),angular=NS(x=0,y=0,z=0)))
        receive_ros_command(controller,msg,10_000_000_000)
        self.assertEqual(controller.desired(),(3.,-4.,0.,17))
        with self.assertRaises(ValueError):receive_ros_command(controller,msg,10_300_000_000)
        self.assertEqual(controller.desired(),(0,0,0,17))
        controller.command(dict(action='left',strength=.2,speed_unit='m/s',client='operator',seq=1))
        msg.seq=2
        with self.assertRaises(RuntimeError):receive_ros_command(controller,msg,10_000_000_000)
        self.assertEqual(controller.desired(),(0,-2.,0.,17))

    def test_fddp_configuration_uses_whole_vehicle_feedback_and_map_predictions(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);web=root/'shaoshao/web_panel';web.mkdir(parents=True)
            robot=yaml.safe_load((ROOT/'web_panel/robot.yaml').read_text())
            robot['navigation']['max_speed_mps']=.5  # Fixture is independent of commissioned vehicle limits.
            (web/'robot.yaml').write_text(yaml.safe_dump(robot))
            runtime=root/'mapping_runtime';runtime.mkdir()
            lio={'/**':{'ros__parameters':{'common':{},'pcd_save':{},'publish':{},'map_filter':{},
                 'mapping':{'extrinsic_T':[-.011,-.02329,.04412],'extrinsic_R':[1,0,0,0,1,0,0,0,1]}}}}
            (runtime/'fast_lio.yaml').write_text(yaml.safe_dump(lio))
            maps=root/'maps';maps.mkdir();(maps/'site.yaml').write_text('image: site.png\nresolution: 0.025\n')
            output=build_navigation_config(root,maps)
            config=yaml.safe_load((output/'nav2.yaml').read_text())
            self.assertNotIn('amcl',config)
            ctrl=config['controller_server']['ros__parameters']['FollowPath']
            self.assertEqual(ctrl['plugin'],'hw_omni_controller::Controller')
            self.assertEqual(ctrl['max_speed_mps'],.5)
            self.assertEqual(ctrl['feedback_timeout_s'],.15)
            self.assertTrue(config['controller_server']['ros__parameters']['enable_stamped_cmd_vel'])
            self.assertEqual(config['hw_obstacle_predictor']['ros__parameters']['range_m'],8.)
            self.assertEqual(config['hw_command_relay']['ros__parameters']['max_speed_mps'],.5)
            local=config['local_costmap']['local_costmap']['ros__parameters']
            self.assertEqual(local['width'],4)
            self.assertEqual(local['global_frame'],'map')
            self.assertEqual(config['controller_server']['ros__parameters']['goal_checker']['plugin'],
                             'nav2_controller::StoppedGoalChecker')

if __name__=='__main__':unittest.main()
