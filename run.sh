#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

cmake -S . -B build
cmake --build build -- -j$(nproc)
echo -e ""
echo "Choose program:"
echo -e ""
echo "Module Tests"
echo "1) Camera test"
echo "2) Depth estimation"
echo "3) Object detection"
echo "4) Servo test"
echo "5) RC receiver test"
echo "6) Motor test"
echo "7) VFH+ controller test (AUTOPILOT)"
echo "8) Follow controller test (FOLLOW)"

echo -e ""

echo "Pipelines"
echo "9) Camera capture and inference"
echo "10) Full Pipeline"

read -p "Select: " choice

if [ "$choice" = "1" ]; then
    ./build/camera_test
elif [ "$choice" = "2" ]; then
    ./build/depth_infer
elif [ "$choice" = "3" ]; then
    ./build/object_detector
elif [ "$choice" = "4" ]; then
    ./build/servo_test
elif [ "$choice" = "5" ]; then
    ./build/rc_receiver_test
elif [ "$choice" = "6" ]; then
    ./build/motor_test
elif [ "$choice" = "7" ]; then
    ./build/vfh_test
elif [ "$choice" = "8" ]; then
    ./build/follow_test
elif [ "$choice" = "9" ]; then
    ./build/pipeline_visual_test
elif [ "$choice" = "10" ]; then
    "$SCRIPT_DIR/venv/bin/python" "$SCRIPT_DIR/oww.py" &
    OWW_PID=$!
    trap "kill $OWW_PID 2>/dev/null; wait $OWW_PID 2>/dev/null" EXIT
    ./build/pipeline
else
    echo "Invalid option"
fi
