#!/bin/bash
systemctl --user stop p2-rviz p2-nav p2-sim 2>/dev/null || true
BASE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec 9>"$BASE/.run.lock"
flock -w 35 9 || { echo 'Previous supervisor is still stopping.'; exit 1; }
echo 'Baseline stopped.'
