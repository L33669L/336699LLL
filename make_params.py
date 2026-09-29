from pathlib import Path
import yaml
src=Path('/opt/ros/jazzy/share/turtlebot3_navigation2/param/burger.yaml')
data=yaml.safe_load(src.read_text())
data['map_server']['ros__parameters']['yaml_filename']='/opt/ros/jazzy/share/turtlebot3_navigation2/map/map.yaml'
data['collision_monitor']['ros__parameters']['scan']['source_timeout']=0.6
data['bt_navigator']['ros__parameters']['default_server_timeout']=1000
data['bt_navigator']['ros__parameters']['error_code_names']=['compute_path_error_code','follow_path_error_code']
data['controller_server']['ros__parameters']['FollowPath']['debug_trajectory_details']=False
Path('burger.baseline.yaml').write_text(yaml.safe_dump(data,sort_keys=False))
rviz=Path('/opt/ros/jazzy/share/turtlebot3_navigation2/rviz/tb3_navigation2.rviz').read_text()
rviz_data=yaml.safe_load(rviz)
rviz_data['Visualization Manager']['Global Options']['Frame Rate']=10
for display in rviz_data['Visualization Manager']['Displays']:
    if display.get('Class')=='rviz_default_plugins/RobotModel':
        display['Enabled']=True
        display['Value']=True
Path('baseline.rviz').write_text(yaml.safe_dump(rviz_data,sort_keys=False))
bridge=yaml.safe_load(Path('/opt/ros/jazzy/share/turtlebot3_gazebo/params/turtlebot3_burger_bridge.yaml').read_text())
for item in bridge:
    if item['ros_topic_name']=='clock': item['ros_topic_name']='clock_raw'
Path('bridge.baseline.yaml').write_text(yaml.safe_dump(bridge,sort_keys=False))
