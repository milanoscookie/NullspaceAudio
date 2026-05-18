#!/bin/bash
# Profile with perf and generate report

set -e

BUILD_DIR="build"
APP="$BUILD_DIR/src/DSPApp"

if [ ! -f "$APP" ]; then
    echo "Error: $APP not found. Build the project first."
    exit 1
fi

echo "=== Profiling with perf ==="
echo "Running application with perf record..."

# Record performance data
sudo perf record -g --call-graph dwarf -F 999 "$APP" "$@"

echo ""
echo "=== Performance Report ==="
sudo perf report --stdio | head -n 50

echo ""
echo "Full interactive report: sudo perf report"
echo "Flame graph: sudo perf script | stackcollapse-perf.pl | flamegraph.pl > flamegraph.svg"
