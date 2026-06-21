#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

cmake -S . -B build
cmake --build build -- -j$(nproc)

echo "Choose program:"
echo "1) Depth estimation"
echo "2) Object detection"
echo "3) Pipeline (T1 camera capture)"
echo "4) Pipeline visual test (depth + detection overlay)"
read -p "Select: " choice

if [ "$choice" = "1" ]; then
    ./build/depth_infer
elif [ "$choice" = "2" ]; then
    ./build/object_detector
elif [ "$choice" = "3" ]; then
    "$SCRIPT_DIR/venv/bin/python" "$SCRIPT_DIR/oww.py" &
    OWW_PID=$!
    trap "kill $OWW_PID 2>/dev/null; wait $OWW_PID 2>/dev/null" EXIT
    ./build/pipeline
elif [ "$choice" = "4" ]; then
    ./build/pipeline_visual_test
else
    echo "Invalid option"
fi
