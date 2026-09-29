import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
rclpy.init(); n=rclpy.create_node('p2_sim_ready'); seen=set(); subs=[]
for key,typ in [('odom',Odometry),('scan',LaserScan),('clock',Clock)]:
    subs.append(n.create_subscription(typ,'/'+key,lambda m,k=key: seen.add(k),qos_profile_sensor_data))
deadline=time.monotonic()+60
while len(seen)<3 and time.monotonic()<deadline: rclpy.spin_once(n,timeout_sec=.2)
n.destroy_node(); rclpy.shutdown()
if len(seen)<3: raise RuntimeError('Missing simulation data: '+str({'odom','scan','clock'}-seen))
print('Simulation odom, scan and clock ready',flush=True)
