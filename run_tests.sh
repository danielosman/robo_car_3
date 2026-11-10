#!/bin/sh
# Host tests (no Pico needed): the link; the shared helpers (time, angles, order
# statistics); PicoB's wheel control, odometry and
# brain (PicoA as PicoB sees it); PicoA's ToF pipeline (tof, tof_rays, tof_format,
# pose's stamp), world map, a move's phases and messages, the actions end to end, watching (change grid, VL53,
# camera, tracker, the frame's turn), body (PicoB as PicoA sees it), the IMU
# from PicoB's odometry to the PC's batches end to end, the
# map on recorded scans (and the window following the robot over the whole-house
# map), camera features, snapshots, the wire (the messages to and
# from the PC, against the golden bytes the PC's test reads too), the link to the
# PC, the commands over it and the loop timer. The fake clock crosses time_us_32()'s wrap in
# every test. Each compiles one module against stubs and fakes and runs it:
# hardware stubs in picoB/app/test/stubs and common/test/stubs, the fake clock
# and fake link in common/test/fakes. The robot's mounting (common/mounting.json)
# is made into build/host/robot_mounting.h first, as the firmware's build does.
# Stops at the first failure.
set -e
cd "$(dirname "$0")"
mkdir -p build
CC="cc -std=c11 -Wall -Wextra -Wdouble-promotion -Werror"
APP="-Icommon/test/fakes -Icommon -Ibuild/host"

# build <name> <source> <include flags...>
build() {
    name=$1 src=$2
    shift 2
    $CC "$@" -o "build/$name" "$src" -lm
}

# The mounting: the header holds the file's numbers (test_mounting); the firmware's
# build makes the same header; a missing or wrong field stops it with its name.
mkdir -p build/host
cmake -DMOUNTING_JSON=common/mounting.json -DMOUNTING_H=build/host/robot_mounting.h -P common/mounting.cmake
if [ -f build/generated/robot_mounting.h ]; then
    cmp build/generated/robot_mounting.h build/host/robot_mounting.h || { echo "the firmware's robot_mounting.h differs (re-run cmake -S . -B build)"; exit 1; }
fi
# refuses <sed edit of mounting.json> <the error it must give>
refuses() {
    sed "$1" common/mounting.json > build/host/bad.json
    if cmake -DMOUNTING_JSON=build/host/bad.json -DMOUNTING_H=build/host/bad.h -P common/mounting.cmake 2> build/host/bad.log \
       || ! tr -s ' \n' ' ' < build/host/bad.log | grep -q "$2"; then
        echo "mounting.cmake: '$1' must stop it with '$2'"; cat build/host/bad.log; exit 1
    fi
}
refuses '/"z_m"/d' 'tof.z_m is missing'
refuses '/"tof"/,/}/d' 'tof.x_m is missing'
refuses 's/"x_m": 0.032/"x_m": "3.2 cm"/' 'tof.x_m must be a number'
refuses 's/"zones_turn_deg": 90/"zones_turn_deg": 45/' 'zones_turn_deg must be 0, 90, 180 or 270'
refuses 's/\[\[-1, 0, 0\]/[[2, 0, 0]/' 'robot_from_sensor.0.0 must be -1, 0 or 1'
refuses 's/"width_px": 160/"width_px": 160.5/' 'camera.width_px must be a whole number'
refuses 's/"width_px": 160/"width_px": 0/' 'camera.width_px must be more than 0'
refuses 's/"z_m": 0.108/"z_m": 0/' 'camera.z_m must be more than 0'
echo "OK: mounting.cmake refuses a missing or wrong field, naming it"
build test_mounting common/test/test_mounting.c $APP
build/test_mounting
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
# The ToF pipeline: reading the sensor, where the zones look, the packed format.
build test_tof picoA/app/test/test_tof.c -IpicoA/app/test/stubs $APP
build/test_tof
build test_tof_rays picoA/app/test/test_tof_rays.c -IpicoA/app/test/stubs $APP
build/test_tof_rays
build test_tof_format picoA/app/test/test_tof_format.c -Icommon
build/test_tof_format
# The map (doc/ARCHITECTURE.md) in simulated rooms; its printed maps go to the log.
build test_cell_map picoA/app/test/test_cell_map.c -IpicoA/app/test/stubs $APP
build/test_cell_map > build/test_cell_map.log || { cat build/test_cell_map.log; exit 1; }
grep -v '^[|+]' build/test_cell_map.log | grep -v '^Map (9\|^up =\|^? no\|^blank'
build test_cell_map_15hz picoA/app/test/test_cell_map.c -DTOF_HZ=15 -IpicoA/app/test/stubs $APP # the rate is the scene's only
build/test_cell_map_15hz > build/test_cell_map_15hz.log || { cat build/test_cell_map_15hz.log; exit 1; }
echo "OK: test_cell_map also at 15 Hz"
# The map on the recorded scans of 10 Oct (picoA/app/test/data): the cup, box, chair, cupboard found.
build test_map_replay picoA/app/test/test_map_replay.c -IpicoA/app/test/stubs $APP
build/test_map_replay
build test_pose picoA/app/test/test_pose.c $APP
build/test_pose
# A move: its phases (still before and after), its messages, its frames.
build test_move picoA/app/test/test_move.c $APP
build/test_move > build/test_move.log || { cat build/test_move.log; exit 1; }
grep '^ok\|passed' build/test_move.log
# The actions end to end (scan, move, turn, watch), each a move, on a simulated robot in a simulated room.
build test_behaviour picoA/app/test/test_behaviour.c -IpicoA/app/test/stubs $APP
build/test_behaviour
build test_behaviour_15hz picoA/app/test/test_behaviour.c -DTOF_HZ=15 -IpicoA/app/test/stubs $APP
build/test_behaviour_15hz > build/test_behaviour_15hz.log || { cat build/test_behaviour_15hz.log; exit 1; }
echo "OK: test_behaviour also at 15 Hz"; grep -A1 "^walkers over" build/test_behaviour_15hz.log
build test_change_grid picoA/app/test/test_change_grid.c -Icommon
build/test_change_grid
build test_watch_tof picoA/app/test/test_watch_tof.c -IpicoA/app/test/stubs $APP
build/test_watch_tof
build test_watch_camera picoA/app/test/test_watch_camera.c -IpicoA/drivers -Icommon -Ibuild/host
build/test_watch_camera
build test_tracker picoA/app/test/test_tracker.c -Icommon
build/test_tracker
build test_watch picoA/app/test/test_watch.c -IpicoA/app/test/stubs -IpicoA/drivers $APP
build/test_watch
build test_loop_statistics picoA/app/test/test_loop_statistics.c
build/test_loop_statistics
build test_body picoA/app/test/test_body.c $APP
build/test_body
# The IMU end to end: PicoB's odometry and brain, the link, PicoA's body and move, the PC's batches.
$CC -IpicoB/app/test/stubs $APP -o build/test_imu_path picoA/app/test/test_imu_path.c picoA/app/test/test_imu_path_picob.c -lm
build/test_imu_path > build/test_imu_path.log || { cat build/test_imu_path.log; exit 1; }
grep -v '^B: \|^Motors\|^PicoA connected\|^Watch 1' build/test_imu_path.log || true
# Camera features (vision) on synthetic frames; its own float/double mixing in the
# test's statistics is allowed (the firmware build checks vision.c with every warning).
build test_vision picoA/app/test/test_vision.c -O2 -Wno-double-promotion -IpicoA/app/test/stubs -IpicoA/drivers $APP
build/test_vision > build/test_vision.log || { cat build/test_vision.log; exit 1; }
grep -v '^shift' build/test_vision.log || true
# Snapshots as the PC reads them; build/test_snapshot.snap is for the PC's parser test.
build test_snapshot picoA/app/test/test_snapshot.c -IpicoA/app/test/stubs -IpicoA/drivers $APP
build/test_snapshot
# The messages to and from the PC: picoA/app/test/data/wire_golden.hex is read by
# pc/robot's wire.test.ts too, so both sides agree on every byte.
build test_wire picoA/app/test/test_wire.c $APP
build/test_wire
build test_link_pc picoA/app/test/test_link_pc.c -IpicoA/app/test/stubs $APP
build/test_link_pc > build/test_link_pc.log || { cat build/test_link_pc.log; exit 1; }
grep -v '^WiFi:\|^Server:\|^Wire:' build/test_link_pc.log
build test_commands picoA/app/test/test_commands.c -IpicoA/app/test/stubs $APP
build/test_commands > build/test_commands.log || { cat build/test_commands.log; exit 1; }
grep '^ok\|passed' build/test_commands.log
