import json, math, time
from pathlib import Path
import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState

# 保留旧导入入口，供 verify.py 使用；采集实现已与初始化解耦。
from navigation.ground_truth import ground_truth

if __name__=='__main__':
    rclpy.init()
    n=rclpy.create_node('p2_initial_pose',parameter_overrides=[Parameter('use_sim_time',value=True)])
    c=n.create_client(GetState,'/amcl/get_state')
    end=time.monotonic()+90
    while time.monotonic()<end:
        if c.wait_for_service(timeout_sec=1):
            f=c.call_async(GetState.Request()); rclpy.spin_until_future_complete(n,f,timeout_sec=2)
            if f.done() and f.result().current_state.id==3: break
        time.sleep(0.5)
    else: raise RuntimeError('AMCL did not become active')
    pub=n.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
    while pub.get_subscription_count()==0 or n.get_clock().now().nanoseconds==0:
        rclpy.spin_once(n,timeout_sec=.1)
        if time.monotonic()>end: raise RuntimeError('Initial pose subscriber/clock unavailable')
    truth=ground_truth()
    # Zero requests the latest available TF, avoiding a future timestamp relative to odom.
    # Run only while stationary, before sending any navigation goal.
    msg=PoseWithCovarianceStamped(); msg.header.frame_id='map'
    msg.pose.pose.position.x=truth['x']; msg.pose.pose.position.y=truth['y']
    msg.pose.pose.orientation.z=math.sin(truth['yaw']/2); msg.pose.pose.orientation.w=math.cos(truth['yaw']/2)
    msg.pose.covariance[0]=.01; msg.pose.covariance[7]=.01; msg.pose.covariance[35]=.01
    pub.publish(msg)
    for _ in range(10): rclpy.spin_once(n,timeout_sec=.1)
    Path('evidence/initial_pose.json').write_text(json.dumps(truth,indent=2))
    print(json.dumps(truth),flush=True)
    n.destroy_node(); rclpy.shutdown()

