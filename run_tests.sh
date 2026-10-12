#!/bin/sh
# Host tests (no Pico needed): the link; the shared helpers (time, angles, order
# statistics); PicoB's wheel control, odometry and
# brain (PicoA as PicoB sees it); PicoA's rangefinder, world map, pose, the
# actions end to end, movement detection (change grid, VL53, camera, tracker), body (PicoB as PicoA sees it), the
# map on recorded scans (and the window following the robot over the whole-house
# map), camera features, the recorder, snapshots, the WiFi console and the loop timer. The fake clock crosses time_us_32()'s wrap in
# every test. Each compiles one module against stubs and fakes and runs it:
# hardware stubs in picoB/app/test/stubs and common/test/stubs, the fake clock
# and fake link in common/test/fakes. Stops at the first failure.
set -e
cd "$(dirname "$0")"
mkdir -p build
CC="cc -std=c11 -Wall -Wextra -Wdouble-promotion -Werror"
APP="-Icommon/test/fakes -Icommon"

# build <name> <source> <include flags...>
build() {
    name=$1 src=$2
    shift 2
    $CC "$@" -o "build/$name" "$src" -lm
}

build link_test common/test/link_test.c -Icommon/test/stubs
build/link_test
build test_helpers common/test/test_helpers.c -Icommon
build/test_helpers
build test_drive picoB/app/test/test_drive.c -IpicoB/app/test/stubs $APP
build/test_drive
build test_odometry picoB/app/test/test_odometry.c -IpicoB/app/test/stubs $APP
build/test_odometry
build test_brain picoB/app/test/test_brain.c -IpicoB/app/test/stubs $APP
build/test_brain
build test_rangefinder picoA/app/test/test_rangefinder.c -IpicoA/app/test/stubs $APP
build/test_rangefinder
# The map (doc/ARCHITECTURE.md) in simulated rooms; its printed maps go to the log.
build test_cell_map picoA/app/test/test_cell_map.c -IpicoA/app/test/stubs $APP
build/test_cell_map > build/test_cell_map.log || { cat build/test_cell_map.log; exit 1; }
grep -v '^[|+]' build/test_cell_map.log | grep -v '^Map (9\|^up =\|^? no\|^blank'
# The map on the recorded scans of 10 Oct (picoA/app/test/data): the cup, box, chair, cupboard found.
build test_map_replay picoA/app/test/test_map_replay.c -IpicoA/app/test/stubs $APP
build/test_map_replay
build test_pose picoA/app/test/test_pose.c $APP
build/test_pose
# The actions end to end (scan, move, turn, watch) on a simulated robot in a simulated room.
build test_behaviour picoA/app/test/test_behaviour.c -IpicoA/app/test/stubs $APP
build/test_behaviour
build test_change_grid picoA/app/test/test_change_grid.c -Icommon
build/test_change_grid
build test_tof_motion picoA/app/test/test_tof_motion.c -IpicoA/app/test/stubs $APP
build/test_tof_motion
build test_camera_motion picoA/app/test/test_camera_motion.c -IpicoA/drivers -Icommon
build/test_camera_motion
build test_tracker picoA/app/test/test_tracker.c -Icommon
build/test_tracker
build test_loop_stats picoA/app/test/test_loop_stats.c
build/test_loop_stats
build test_body picoA/app/test/test_body.c $APP
build/test_body
build test_recorder picoA/app/test/test_recorder.c -IpicoA/app/test/stubs $APP
build/test_recorder
# Camera features (vision) on synthetic frames; its own float/double mixing in the
# test's statistics is allowed (the firmware build checks vision.c with every warning).
build test_vision picoA/app/test/test_vision.c -O2 -Wno-double-promotion -IpicoA/app/test/stubs -IpicoA/drivers $APP
build/test_vision | grep -v '^shift'
# Snapshots as the PC reads them; build/test_snapshot.snap is for the PC's parser test.
build test_snapshot picoA/app/test/test_snapshot.c -IpicoA/app/test/stubs -IpicoA/drivers $APP
build/test_snapshot
build test_wifi_console picoA/app/test/test_wifi_console.c -IpicoA/app/test/stubs $APP
build/test_wifi_console > build/test_wifi_console.log || { cat build/test_wifi_console.log; exit 1; }
grep -v '^WiFi:\|^Server:' build/test_wifi_console.log
