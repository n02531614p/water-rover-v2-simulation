#!/usr/bin/env bash
# Host checks for the integrated firmware (needs g++ only):
#   1. unit tests of every core (pump, motion, mission, power)
#   2. compile check of waterdroid_esp32.ino against Arduino/ESP32 stubs
#   3. the power_core.h copies in the repo are identical
set -euo pipefail
cd "$(dirname "$0")"
OUT="${TMPDIR:-/tmp}"
g++ -std=c++11 -Wall -Wextra -O1 test_cores.cpp -o "$OUT/test_cores"
"$OUT/test_cores"
g++ -std=c++11 -Wall -Wextra -fsyntax-only -include stubs/arduino_stub.h -x c++ -I stubs ../waterdroid_esp32/waterdroid_esp32.ino
echo "waterdroid_esp32.ino: compile check OK"
for f in ../../filtration_monitor-1.ino ../../robotic_arm.ino; do
  g++ -std=c++11 -fsyntax-only -include stubs/arduino_stub.h -x c++ -I stubs "$f"
  g++ -DESP32 -std=c++11 -fsyntax-only -include stubs/arduino_stub.h -x c++ -I stubs "$f"
done
PM=../../power-management/power-management/esp32/power_management
g++ -std=c++11 -fsyntax-only -include stubs/arduino_stub.h -x c++ -I stubs -I "$PM" "$PM/power_management.ino"
echo "standalone sketches: compile check OK"
REF=../waterdroid_esp32/src/power_core.h
for f in ../../power-management/power-management/core/power_core.h ../../power-management/power-management/esp32/power_management/power_core.h; do
  cmp -s "$REF" "$f" || { echo "power_core.h out of sync: $f"; exit 1; }
done
echo "power_core.h copies in sync"
