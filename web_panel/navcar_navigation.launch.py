from pathlib import Path
import os
import yaml
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    root=Path(os.environ.get('SHAOSHAO_HW_RUNTIME',str(Path.home()/'ccc/shaoshao-hw-runtime')))
    runtime=root/'navigation_runtime';params=str(runtime/'nav2.yaml')
    frames=yaml.safe_load((runtime/'frames.yaml').read_text(encoding='utf-8'))
    config=str(root/'shaoshao/web_panel/robot.yaml');maps=frames['maps_dir']
    import sys
    sys.path.insert(0,str(root/'shaoshao'))
    from shaoshao_hw.config import load
    registration=load(config)['navigation']['registration']
    nodes=[Node(package='fast_lio',executable='fastlio_mapping',name='navigation_lio',
                parameters=[str(runtime/'fast_lio.yaml')],
                remappings=[('/Odometry','/navcar/state_estimation')],output='screen'),
           Node(package='pointcloud_to_laserscan',executable='pointcloud_to_laserscan_node',
                name='navigation_scan',parameters=[params],
                remappings=[('cloud_in','/cloud_registered_body'),('scan','/navcar/scan')],output='screen'),
           Node(package='hw_registration',executable='registration_server',name='hw_registration',
                parameters=[{'map_path':str(Path(maps)/'site.pcd'),'voxel_m':float(registration['voxel_m']),'threads':2}],output='screen'),
           Node(package='shaoshao_hw_runtime',executable='localizer',name='hw_localizer',
                parameters=[{'config_path':config,'maps_dir':maps}],output='screen')]
    for transform in frames['static_transforms']:
        x,y,z=transform['xyz']
        nodes.append(Node(package='tf2_ros',executable='static_transform_publisher',
            name='navigation_'+transform['child'],arguments=['--frame-id','navcar_imu',
                '--child-frame-id',transform['child'],'--x',str(x),'--y',str(y),'--z',str(z),
                '--yaw','0','--pitch','0','--roll','0']))
    for package,executable,name in [('nav2_map_server','map_server','map_server'),
        ('nav2_planner','planner_server','planner_server'),('nav2_controller','controller_server','controller_server'),
        ('nav2_bt_navigator','bt_navigator','bt_navigator')]:
        nodes.append(Node(package=package,executable=executable,name=name,parameters=[params],
            remappings=[('cmd_vel','/navcar/cmd_vel_auto')],output='screen'))
    nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='navigation_lifecycle',
        parameters=[{'autostart':False,'use_sim_time':False,
        'node_names':['map_server','planner_server','controller_server','bt_navigator']}],output='screen'))
    handlers=[RegisterEventHandler(OnProcessExit(target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='Required HW navigation process exited'))])) for node in nodes]
    return LaunchDescription(handlers+nodes)
