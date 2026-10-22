from pathlib import Path
import math
import os
import yaml

SOURCE=Path(__file__).resolve().parent.parent
RUNTIME=Path(os.environ.get('SHAOSHAO_HW_RUNTIME',str(Path.home()/'ccc/shaoshao-hw-runtime')))

DEFAULTS={
 'mpc':{'tau_x_s':.30,'tau_y_s':.30,'feedback_timeout_s':.15,
        'prediction_timeout_s':.30,'obstacle_margin_m':.15},
 'obstacles':{'resolution_m':.05,'range_m':8.,'static_distance_m':.15,
              'association_gate_m':.8,'confirmations':3,'lost_timeout_s':.5,'velocity_decay_s':2.},
 'registration':{'period_s':1.,'voxel_m':.05,'max_error':.30,'min_overlap':.80,
                 'min_eigenvalue':1000.,'max_condition':1e6,'max_step_m':.2,
                 'max_step_rad':.1,'alpha':.4,'confirmations':3,
                 'fresh_s':1.6,'timeout_s':2.,'hold_distance_m':.05,'degraded_speed_mps':.05,
                 'search_timeout_s':30.,'retry_s':5.,'scan_timeout_s':.5},
 'motion':{'acceleration_mps2':.3,'deceleration_mps2':.3,'jerk_mps3':1.,
           'frequency_hz':50.,'command_timeout_s':.2},
 'index':{'distance_m':.25,'angle_rad':math.radians(10),'rings':20,'sectors':60,'radius_m':15.}}

def load(path=None):
    path=Path(path or SOURCE/'web_panel/robot.yaml')
    c=yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(c,dict):raise ValueError('robot.yaml must be a mapping')
    nav=c.setdefault('navigation',{})
    for section,defaults in DEFAULTS.items():
        values={**defaults,**nav.get(section,{})}
        if set(values)!=set(defaults):raise ValueError('Unknown '+section+' setting')
        for key,value in values.items():
            if type(value) not in (int,float) or not math.isfinite(value) or value<=0:
                raise ValueError('Invalid '+section+'.'+key)
        nav[section]=values
    r=nav.setdefault('map_resolution_m',.025)
    if type(r) not in (float,int) or not .01<=r<=.1:raise ValueError('Invalid map resolution')
    if not .01<=nav.get('max_speed_mps',.5)<=.5:raise ValueError('Speed must be <= 0.5 m/s')
    reg=nav['registration']
    mpc=nav['mpc'];obstacles=nav['obstacles']
    if not (.02<=mpc['tau_x_s']<=3 and .02<=mpc['tau_y_s']<=3 and
            mpc['feedback_timeout_s']<=.2 and mpc['prediction_timeout_s']<=.5):
        raise ValueError('Invalid MPC model/freshness bounds')
    if not (.02<=obstacles['resolution_m']<=.2 and 1<=obstacles['range_m']<=15 and
            type(obstacles['confirmations']) is int and 2<=obstacles['confirmations']<=10):
        raise ValueError('Invalid obstacle settings')
    if not (reg['fresh_s']<reg['timeout_s']<=5 and reg['degraded_speed_mps']<=.1 and
            reg['alpha']<=1 and reg['min_overlap']<=1 and reg['hold_distance_m']<=.2):
        raise ValueError('Invalid registration safety bounds')
    if type(reg['confirmations']) is not int or not 2<=reg['confirmations']<=10:
        raise ValueError('Registration confirmations must be 2..10')
    if nav['motion']['frequency_hz']!=50 or nav['motion']['command_timeout_s']>.2:
        raise ValueError('Motion frequency/timeout must preserve watchdog contract')
    return c
