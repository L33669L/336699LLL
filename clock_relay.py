import rclpy
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy
qos_profile_clock=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
rclpy.init(); n=rclpy.create_node('p2_clock_relay')
pub=n.create_publisher(Clock,'/clock',qos_profile_clock); last=None
def receive(msg):
    global last
    ns=msg.clock.sec*1000000000+msg.clock.nanosec
    if last is None or ns<last or ns-last>=20000000:
        pub.publish(msg); last=ns
sub=n.create_subscription(Clock,'/clock_raw',receive,qos_profile_clock)
try: rclpy.spin(n)
except KeyboardInterrupt: pass
finally:
    n.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
