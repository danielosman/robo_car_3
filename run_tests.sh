#!/bin/sh
# Host tests (no Pico needed): the link, and PicoB's wheel control and odometry.
# Each compiles one module against stubs in its test/ folder and runs it.
set -e
cd "$(dirname "$0")"
mkdir -p build
CC="cc -std=c11 -Wall -Wextra -Werror"
$CC -Icommon/test/stubs -o build/link_test common/test/link_test.c && build/link_test
$CC -IpicoB/app/test/stubs -o build/test_drive picoB/app/test/test_drive.c -lm && build/test_drive
$CC -IpicoB/app/test/stubs -o build/test_odometry picoB/app/test/test_odometry.c -lm && build/test_odometry
