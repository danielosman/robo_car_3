#!/bin/sh
# Replay (doc/TELEMETRY_PLAN.md §5): a recorded take through the robot's map code, both
# variants side by side (tools/replay_map.c). Needs the take in DuckDB (pc/robot:
# npm run import).
#   tools/replay.sh <take_id> [floor margin, default 0.9]
set -e
cd "$(dirname "$0")/.."
[ -n "$1" ] || { echo "usage: tools/replay.sh <take_id> [floor margin]"; exit 2; }
mkdir -p build/replay
cc -std=c11 -O2 -Wall -Wextra -IpicoA/app/test/stubs -Icommon/test/fakes -Icommon -IpicoA/app \
   -o build/replay/replay_map tools/replay_map.c -lm
(cd pc/robot && node src/frames.ts "$1" "../../build/replay/$1.frames" >/dev/null)
build/replay/replay_map "build/replay/$1.frames" "${2:-0.9}"
