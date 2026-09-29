import json,time,sys,math
from pathlib import Path
import rclpy
from lifecycle_msgs.srv import GetState
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data
from bond.msg import Status
rclpy.init(); n=rclpy.create_node('p2_stability_check')
rows=[]; count={'clock':0,'odom':0}; latest={}
bond_seen={}; bond_gaps={}; clock_samples=[]
def heartbeat(m):
    key=m.id+':'+m.instance_id
    now=time.monotonic()
    if key in bond_seen: bond_gaps[key]=max(bond_gaps.get(key,0),now-bond_seen[key])
    bond_seen[key]=now
def receive(k,m): count[k]+=1; latest[k]=m
subs=[n.create_subscription(Clock,'/clock',lambda m:receive('clock',m),qos_profile_sensor_data),n.create_subscription(Odometry,'/odom',lambda m:receive('odom',m),qos_profile_sensor_data)]
subs.append(n.create_subscription(Status,'/bond',heartbeat,100))
clients={k:n.create_client(GetState,'/'+k+'/get_state') for k in ['amcl','controller_server','planner_server','route_server','bt_navigator','smoother_server','behavior_server','collision_monitor','docking_server','global_costmap/global_costmap','local_costmap/local_costmap']}
managers={k:n.create_client(Trigger,'/lifecycle_manager_'+k+'/is_active') for k in ['localization','navigation']}
start=time.monotonic()
duration=int(sys.argv[1]) if len(sys.argv)>1 else 120
samples=math.ceil(duration/20)+1
for i in range(samples):
    end=start+20*i
    while time.monotonic()<end: rclpy.spin_once(n,timeout_sec=.1)
    row={'elapsed':round(time.monotonic()-start,2),'counts':dict(count),'states':{},'managers':{}}
    for k,c in clients.items():
        if not c.wait_for_service(timeout_sec=3): row['states'][k]='unavailable'; continue
        f=c.call_async(GetState.Request()); rclpy.spin_until_future_complete(n,f,timeout_sec=5)
        row['states'][k]=f.result().current_state.label if f.done() else 'timeout'
    for k,c in managers.items():
        if not c.wait_for_service(timeout_sec=3): row['managers'][k]=False; continue
        f=c.call_async(Trigger.Request()); rclpy.spin_until_future_complete(n,f,timeout_sec=5)
        row['managers'][k]=bool(f.done() and f.result().success)
    if 'odom' in latest: row['velocity']=[latest['odom'].twist.twist.linear.x,latest['odom'].twist.twist.angular.z]
    rows.append(row); print(json.dumps(row),flush=True)
    if not all(v=='active' for v in row['states'].values()) or not all(row['managers'].values()): break
passed=len(rows)==samples and all(all(v=='active' for v in r['states'].values()) and all(r['managers'].values()) for r in rows)
passed=passed and all(rows[i]['counts']['clock']>rows[i-1]['counts']['clock'] and rows[i]['counts']['odom']>rows[i-1]['counts']['odom'] for i in range(1,len(rows)))
Path('evidence/stability.json').write_text(json.dumps({'passed':passed,'duration_wall_seconds':round(time.monotonic()-start,2),'max_bond_gap_wall_seconds':bond_gaps,'rows':rows},indent=2))
n.destroy_node(); rclpy.shutdown()
raise SystemExit(0 if passed else 1)
