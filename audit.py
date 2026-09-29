import json,os,subprocess,time
from pathlib import Path
import yaml
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.utilities import get_rmw_implementation_identifier
rclpy.init(); n=rclpy.create_node('p2_baseline_audit')
start=time.monotonic()
while time.monotonic()-start<3: rclpy.spin_once(n,timeout_sec=.1)
result={'rmw':get_rmw_implementation_identifier(),'topics':{},'parameters':{}}
for topic in ['/scan','/odom','/clock','/cmd_vel','/cmd_vel_nav','/cmd_vel_smoothed','/particle_cloud']:
    def info(e): return {'node':e.node_name,'type':e.topic_type,'reliability':str(e.qos_profile.reliability),'durability':str(e.qos_profile.durability)}
    result['topics'][topic]={'publishers':[info(e) for e in n.get_publishers_info_by_topic(topic)],'subscribers':[info(e) for e in n.get_subscriptions_info_by_topic(topic)]}
for node,names in {'amcl':['use_sim_time'],'controller_server':['use_sim_time','goal_checker.xy_goal_tolerance','FollowPath.debug_trajectory_details'],'bt_navigator':['use_sim_time','default_server_timeout'],'collision_monitor':['use_sim_time','scan.source_timeout'],'map_server':['yaml_filename','use_sim_time'],'lifecycle_manager_navigation':['bond_timeout']}.items():
    c=n.create_client(GetParameters,'/'+node+'/get_parameters')
    if not c.wait_for_service(timeout_sec=5): result['parameters'][node]='unavailable'; continue
    req=GetParameters.Request(); req.names=names; f=c.call_async(req); rclpy.spin_until_future_complete(n,f,timeout_sec=5)
    result['parameters'][node]={k:str(v) for k,v in zip(names,f.result().values)} if f.done() else 'timeout'
Path('evidence/final_audit.json').write_text(json.dumps(result,indent=2))
def flat(d,p=''):
    out={}
    for k,v in d.items():
        if isinstance(v,dict): out.update(flat(v,p+k+'.'))
        else: out[p+k]=v
    return out
a=flat(yaml.safe_load(Path('evidence/burger.original.yaml').read_text())); b=flat(yaml.safe_load(Path('burger.baseline.yaml').read_text()))
changes={k:{'before':a.get(k),'after':b.get(k)} for k in a.keys()|b.keys() if a.get(k)!=b.get(k)}
Path('evidence/parameter_changes.json').write_text(json.dumps(changes,indent=2))
print(json.dumps({'rmw':result['rmw'],'changes':changes},indent=2))
n.destroy_node(); rclpy.shutdown()
