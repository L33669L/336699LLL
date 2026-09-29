#!/bin/bash
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
BASE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec ros2 launch "$BASE/sim.launch.py"
