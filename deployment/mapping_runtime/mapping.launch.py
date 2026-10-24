from pathlib import Path
import yaml
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    root=Path(os.environ.get('SHAOSHAO_HW_RUNTIME',str(Path.home()/'ccc/shaoshao-hw-runtime')))
    lio=yaml.safe_load((root/'mapping_runtime/fast_lio.yaml').read_text(encoding='utf-8'))
    resolution=lio['/**']['ros__parameters'].get('map_filter',{}).get('static_resolution',.025)
    # All consumers must use the same launch snapshot. The current-directory
    # marker can still name preview while the parent starts this process.
    map_file=lio['/**']['ros__parameters'].get('map_file_path')
    maps=(Path(map_file).expanduser().resolve().parent if map_file else
          Path((root/'current_mapping_dir').read_text(encoding='utf-8').strip()))
    nodes=[
      Node(package='shaoshao_hw_runtime',executable='index_recorder',parameters=[{'maps_dir':str(maps),'config_path':str(root/'shaoshao/web_panel/robot.yaml')}],output='screen'),
      Node(package='fast_lio',executable='fastlio_mapping',output='screen',parameters=[str(root/'mapping_runtime/fast_lio.yaml')],remappings=[('/Odometry','/state_estimation')]),
      Node(package='terrain_analysis_ext',executable='terrainAnalysisExt',output='screen',parameters=[{
        'map_frame':'odom','scanVoxelSize':.1,'decayTime':.5,'noDecayDis':0.,'clearingDis':30.,
        'useSorting':True,'quantileZ':.25,'vehicleHeight':1.5,'voxelPointUpdateThre':100,'voxelTimeUpdateThre':2.,
        'lowerBoundZ':-3.,'upperBoundZ':1.,'disRatioZ':.1,'checkTerrainConn':False,'localTerrainMapRadius':0.,
        'vehicleLength':.7,'vehicleWidth':.7}],remappings=[('/registered_scan','/cloud_registered')]),
      Node(package='tf2_ros',executable='static_transform_publisher',name='mapping_map_odom',arguments=['--frame-id','map','--child-frame-id','odom','--qx','0','--qw','1']),
      Node(package='fast_lio',executable='static_map_projector',output='screen',parameters=[{'resolution':float(resolution)}]),
    ]
    return LaunchDescription(nodes)
