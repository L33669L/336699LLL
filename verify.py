import json, math, sys, time, atexit, subprocess
from pathlib import Path
from collections import defaultdict
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path as NavPath, OccupancyGrid
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
from initial_pose import ground_truth
navigation_guard={'armed':False}
def stop_failed_navigation():
    if navigation_guard['armed']:
        subprocess.run(['systemctl','--user','stop','p2-nav'],timeout=25,check=False)
atexit.register(stop_failed_navigation)

rclpy.init()
n=rclpy.create_node('p2_verify',parameter_overrides=[Parameter('use_sim_time',value=True)])
counts=defaultdict(int); latest={}; odoms=[]; commands=[]; paths=[]; stamps=defaultdict(list)
def cb(key):
    def receive(m):
        counts[key]+=1; latest[key]=m
        if hasattr(m,'header'): stamps[key].append(m.header.stamp.sec+m.header.stamp.nanosec/1e9)
        if key=='odom': odoms.append((time.monotonic(),m.pose.pose.position.x,m.pose.pose.position.y,m.twist.twist.linear.x,m.twist.twist.angular.z))
        if key=='cmd_vel': commands.append((time.monotonic(),m.twist.linear.x,m.twist.angular.z))
        if key=='plan': paths.append(len(m.poses))
    return receive
subs=[]
for key,typ,topic in [('scan',LaserScan,'/scan'),('odom',Odometry,'/odom'),('clock',Clock,'/clock'),('cmd_vel',TwistStamped,'/cmd_vel'),('plan',NavPath,'/plan'),('amcl_pose',PoseWithCovarianceStamped,'/amcl_pose'),('global_costmap',OccupancyGrid,'/global_costmap/costmap'),('local_costmap',OccupancyGrid,'/local_costmap/costmap')]:
    subs.append(n.create_subscription(typ,topic,cb(key),qos_profile_sensor_data))
buf=Buffer(); listener=TransformListener(buf,n)
def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end: rclpy.spin_once(n,timeout_sec=.1)
def call(typ,name):
    c=n.create_client(typ,name)
    if not c.wait_for_service(timeout_sec=8): return None
    for _ in range(2):
        f=c.call_async(typ.Request()); rclpy.spin_until_future_complete(n,f,timeout_sec=8)
        if f.done(): return f.result()
    return None
spin(10)
report={'lifecycle':{},'managers':{},'tf':{}}
for name in ['map_server','amcl','controller_server','smoother_server','planner_server','route_server','behavior_server','bt_navigator','waypoint_follower','velocity_smoother','collision_monitor','docking_server','global_costmap/global_costmap','local_costmap/local_costmap']:
    r=call(GetState,'/'+name+'/get_state'); report['lifecycle'][name]=r.current_state.label if r else 'UNAVAILABLE'
for name in ['localization','navigation']:
    r=call(Trigger,'/lifecycle_manager_'+name+'/is_active'); report['managers'][name]=bool(r and r.success)
for a,b in [('map','odom'),('odom','base_footprint'),('base_footprint','base_link'),('base_link','base_scan'),('map','base_link')]:
    try:
        t=buf.lookup_transform(a,b,rclpy.time.Time()); report['tf'][a+'->'+b]={'x':t.transform.translation.x,'y':t.transform.translation.y,'z':t.transform.translation.z}
    except Exception as e: report['tf'][a+'->'+b]=str(e)
report['counts_before']=dict(counts)
report['ground_truth_before']=ground_truth()
report['ready']=all(v=='active' for v in report['lifecycle'].values()) and all(report['managers'].values()) and all(counts[k]>0 for k in ['scan','odom','clock','global_costmap','local_costmap']) and all(isinstance(v,dict) for v in report['tf'].values())
print(json.dumps(report,indent=2),flush=True)
if '--navigate' in sys.argv:
    if not all(v=='active' for v in report['lifecycle'].values()) or not all(report['managers'].values()): raise RuntimeError('Lifecycle not ready')
    if not all(counts[k]>0 for k in ['scan','odom','clock','global_costmap','local_costmap']): raise RuntimeError('Data chain not ready')
    # This goal is specific to the default TurtleBot3 world/map and fresh spawn.
    goal=NavigateToPose.Goal(); goal.pose.header.frame_id='map'; goal.pose.header.stamp=n.get_clock().now().to_msg()
    goal.pose.pose.position.x=-1.3; goal.pose.pose.position.y=-.5; goal.pose.pose.orientation.w=1.
    client=ActionClient(n,NavigateToPose,'/navigate_to_pose')
    if not client.wait_for_server(timeout_sec=10): raise RuntimeError('Action unavailable')
    navigation_guard['armed']=True
    f=client.send_goal_async(goal); rclpy.spin_until_future_complete(n,f,timeout_sec=10)
    h=f.result(); report['goal']={'x':-1.3,'y':-.5,'accepted':h.accepted}
    print('Goal accepted: '+str(h.accepted),flush=True)
    if not h.accepted: raise RuntimeError('Goal rejected')
    result=h.get_result_async(); deadline=time.monotonic()+150
    while not result.done() and time.monotonic()<deadline: rclpy.spin_once(n,timeout_sec=.1)
    if not result.done():
        cf=h.cancel_goal_async(); rclpy.spin_until_future_complete(n,cf,timeout_sec=5)
        rclpy.spin_until_future_complete(n,result,timeout_sec=10)
        report['timeout']=True
    if result.done():
        rr=result.result(); report['action_status']=rr.status; report['action_result']=str(rr.result)
    stopped_at=time.monotonic(); spin(10)
    report['ground_truth_after']=ground_truth()
    report['path_messages']=len(paths); report['max_path_poses']=max(paths,default=0)
    report['nonzero_cmd_messages']=sum(abs(v)>1e-4 or abs(w)>1e-4 for _,v,w in commands)
    report['odom_displacement']=math.hypot(odoms[-1][1]-odoms[0][1],odoms[-1][2]-odoms[0][2])
    tail=[o for o in odoms if o[0]>time.monotonic()-3]
    report['stop_tail_max_linear']=max((abs(o[3]) for o in tail),default=None)
    report['stop_tail_max_angular']=max((abs(o[4]) for o in tail),default=None)
    report['last_cmd']=commands[-1][1:] if commands else None
    p=report['ground_truth_after']; report['truth_goal_distance']=math.hypot(p['x']+1.3,p['y']+.5)
    report['lifecycle_after']={}
    for name in report['lifecycle']:
        r=call(GetState,'/'+name+'/get_state'); report['lifecycle_after'][name]=r.current_state.label if r else 'UNAVAILABLE'
    report['functional_pass']=report.get('action_status')==4 and report['max_path_poses']>1 and report['odom_displacement']>.1 and report['stop_tail_max_linear'] is not None and report['stop_tail_max_linear']<.01 and report['stop_tail_max_angular']<.01 and all(v=='active' for v in report['lifecycle_after'].values())
    navigation_guard['armed']=not report['functional_pass']
report['counts']=dict(counts)
report['stamp_rates']={k:(len(v)-1)/(v[-1]-v[0]) for k,v in stamps.items() if len(v)>1 and v[-1]>v[0]}
report['scan_frame']=latest['scan'].header.frame_id if 'scan' in latest else None
report['scan_finite_ranges']=sum(math.isfinite(x) for x in latest['scan'].ranges) if 'scan' in latest else 0
Path('evidence/verification'+('_navigation' if '--navigate' in sys.argv else '')+'.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2),flush=True)
n.destroy_node(); rclpy.shutdown()
if not report['ready'] or ('--navigate' in sys.argv and not report.get('functional_pass',False)): sys.exit(1)
