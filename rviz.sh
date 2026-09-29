#!/bin/bash
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
BASE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec ros2 run rviz2 rviz2 -d "$BASE/baseline.rviz" --ros-args -p use_sim_time:=true
