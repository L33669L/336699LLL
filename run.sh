#!/bin/bash
set -eo pipefail
BASE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$BASE"
source "$BASE/env.sh"
mkdir -p evidence logs
exec 9>"$BASE/.run.lock"
flock -n 9 || { echo 'Baseline is already running. Use a second terminal for tests.'; exit 1; }
cleanup() {
  systemctl --user stop p2-rviz p2-nav p2-sim 2>/dev/null || true
  journalctl --user -u p2-nav -u p2-sim -u p2-rviz --since "$STARTED" --no-pager > "logs/run-$(date +%Y%m%d-%H%M%S).log"
}
STARTED=$(date '+%Y-%m-%d %H:%M:%S')
trap cleanup EXIT
trap 'exit 130' INT TERM
systemctl --user stop p2-rviz p2-nav p2-sim 2>/dev/null || true
systemctl --user reset-failed p2-rviz p2-nav p2-sim 2>/dev/null || true
ros2 daemon stop >/dev/null 2>&1 || true
systemd-run --user --unit=p2-sim --property=KillSignal=SIGINT --property=TimeoutStopSec=15 --property=WorkingDirectory="$BASE" /bin/bash "$BASE/sim.sh"
python3 ready.py
systemd-run --user --unit=p2-nav --property=KillSignal=SIGINT --property=TimeoutStopSec=15 --property=WorkingDirectory="$BASE" /bin/bash "$BASE/nav.sh"
python3 initial_pose.py
systemd-run --user --unit=p2-rviz --property=KillSignal=SIGINT --property=TimeoutStopSec=15 /bin/bash "$BASE/rviz.sh"
python3 verify.py
echo 'Baseline ready. Keep this terminal open. Ctrl+C stops the entire baseline.'
while systemctl --user is-active --quiet p2-sim && systemctl --user is-active --quiet p2-nav; do
  sleep 20
  python3 health.py --quiet || { echo 'Lifecycle health check failed; stopping the entire baseline.'; exit 1; }
done
echo 'A baseline service stopped. Inspect logs.'
exit 1
