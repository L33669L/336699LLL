import rclpy,sys,os
from std_srvs.srv import Trigger
rclpy.init(); n=rclpy.create_node('p2_health_guard'); ok=True; reason=''
for name in ['localization','navigation']:
    c=n.create_client(Trigger,'/lifecycle_manager_'+name+'/is_active')
    if not c.wait_for_service(timeout_sec=3): ok=False; reason=name+' manager unavailable'; break
    f=c.call_async(Trigger.Request()); rclpy.spin_until_future_complete(n,f,timeout_sec=5)
    if not (f.done() and f.result().success): ok=False; reason=name+' manager inactive or response timeout'; break
n.destroy_node(); rclpy.shutdown()
if '--quiet' not in sys.argv:
    print('HEALTHY: localization and navigation are active.' if ok else 'NOT READY: '+reason+'. Start run.sh and source env.sh before checking.')
    print('RMW_IMPLEMENTATION='+os.environ.get('RMW_IMPLEMENTATION','system default'))
raise SystemExit(0 if ok else 1)
