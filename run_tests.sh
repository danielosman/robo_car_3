#!/bin/sh
# Host tests (no Pico needed): the link; PicoB's wheel control, odometry and
# brain (PicoA as PicoB sees it); PicoA's rangefinder, world map, pose, the
# start-up scan end to end, body (PicoB as PicoA sees it) and robot tests. Each compiles one module against stubs and fakes and runs it:
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
build test_drive picoB/app/test/test_drive.c -IpicoB/app/test/stubs $APP
build/test_drive
build test_odometry picoB/app/test/test_odometry.c -IpicoB/app/test/stubs $APP
build/test_odometry
build test_brain picoB/app/test/test_brain.c -IpicoB/app/test/stubs $APP
build/test_brain
build test_rangefinder picoA/app/test/test_rangefinder.c -IpicoA/app/test/stubs $APP
build/test_rangefinder
build test_world_map picoA/app/test/test_world_map.c -IpicoA/app/test/stubs $APP
build/test_world_map
build test_pose picoA/app/test/test_pose.c $APP
build/test_pose
# The start-up scan end to end; prints the map it made of the simulated room.
build test_behaviour picoA/app/test/test_behaviour.c -IpicoA/app/test/stubs $APP
build/test_behaviour
build test_body picoA/app/test/test_body.c $APP
build/test_body
build test_robot_test picoA/app/test/test_robot_test.c $APP
# The drift test prints a progress line every 15 s of simulated time: keep them
# out of the way, but show everything if the test fails.
build/test_robot_test > build/test_robot_test.log || { cat build/test_robot_test.log; exit 1; }
grep -v '^Drift [0-9]' build/test_robot_test.log
