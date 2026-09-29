from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter

def generate_launch_description():
    base=Path(__file__).resolve().parent
    bringup=Path(get_package_share_directory('nav2_bringup'))/'launch/bringup_launch.py'
    return LaunchDescription([GroupAction([
        SetParameter(name='bond_timeout',value=15.0),
        SetParameter(name='bond_heartbeat_period',value=0.5),
        SetParameter(name='attempt_respawn_reconnection',value=False),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup)),launch_arguments={
            'use_sim_time':'True','use_composition':'False',
            'map':'/opt/ros/jazzy/share/turtlebot3_navigation2/map/map.yaml',
            'params_file':str(base/'burger.baseline.yaml'),
        }.items()),
    ])])
