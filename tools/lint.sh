#!/bin/sh
# Linters: cppcheck on the firmware's C (PicoA, PicoB, common, their host tests;
# not the vendor libraries or bring-up code) and ESLint plus tsc on the PC server
# (pc/robot). Needs `brew install cppcheck` and `npm install` in pc/robot.
# Fails on the first finding. Run from anywhere: tools/lint.sh
set -e
cd "$(dirname "$0")/.."

# assertWithSideEffect: the host tests assert on calls to the code under test; they
# are never built without asserts.
cppcheck --enable=warning,performance,portability --std=c11 --inline-suppr --error-exitcode=1 --quiet -j8 \
    --suppress=missingIncludeSystem --suppress=missingInclude \
    --suppress='assertWithSideEffect:*/test/*' \
    -Icommon -IpicoA/app -IpicoB/app -IpicoA/drivers -IpicoB/drivers \
    -i picoA/drivers/lib -i picoA/bringup -i picoB/bringup \
    --template='{file}:{line}: {severity} {id}: {message}' \
    picoA picoB common
echo "cppcheck: clean"

cd pc/robot
npm run --silent lint
npm run --silent typecheck
echo "eslint, tsc: clean"
