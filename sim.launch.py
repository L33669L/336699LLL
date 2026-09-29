import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    base=Path(__file__).resolve().parent
    tb=Path(get_package_share_directory('turtlebot3_gazebo'))
    gz=Path(get_package_share_directory('ros_gz_sim'))
    return LaunchDescription([
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',str(tb/'models')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gz/'launch/gz_sim.launch.py')),launch_arguments={'gz_args':'-r -s -v2 '+str(tb/'worlds/turtlebot3_world.world'),'on_exit_shutdown':'true'}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gz/'launch/gz_sim.launch.py')),launch_arguments={'gz_args':'-g -v2','on_exit_shutdown':'true'}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(tb/'launch/robot_state_publisher.launch.py')),launch_arguments={'use_sim_time':'true'}.items()),
        Node(package='ros_gz_sim',executable='create',arguments=['-name','burger','-file',str(tb/'models/turtlebot3_burger/model.sdf'),'-x','-2.0','-y','-0.5','-z','0.01'],output='screen'),
        Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[{'config_file':str(base/'bridge.baseline.yaml')}],output='screen'),
        ExecuteProcess(cmd=['python3',str(base/'clock_relay.py')],output='screen'),
    ])
