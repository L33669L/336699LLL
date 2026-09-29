#!/bin/bash
set -e
BASE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$BASE"
source "$BASE/env.sh"
python3 health.py
