#!/usr/bin/env bash
# Build the simulator, run every scenario, validate each CSV and plot it.
#
# Usage: scripts/run_all.sh [build-dir] [output-dir]
#   build-dir   CMake build directory (default: build)
#   output-dir  where CSVs and plots go (default: output)
#
# Python: uses $PYTHON if set, else .venv/bin/python if present, else python3.

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${1:-build}"
output_dir="${2:-output}"

if [[ -n "${PYTHON:-}" ]]; then
  python="$PYTHON"
elif [[ -x "$repo_root/.venv/bin/python" ]]; then
  python="$repo_root/.venv/bin/python"
else
  python="python3"
fi

cd "$repo_root"

if [[ ! -f "$build_dir/CMakeCache.txt" ]]; then
  cmake -S . -B "$build_dir"
fi
cmake --build "$build_dir" --target vehiclesim

simulator="$build_dir/vehiclesim"
mkdir -p "$output_dir"

csv_files=()
for scenario in scenarios/*.json; do
  name="$(basename "$scenario" .json)"
  csv="$output_dir/$name.csv"
  echo "==> $name"
  "$simulator" "$scenario" --output "$csv"
  "$python" scripts/validate_csv.py "$csv" --scenario "$scenario"
  csv_files+=("$csv")
done

echo "==> plots"
"$python" scripts/visualize.py "${csv_files[@]}" --output-dir "$output_dir/plots"

# A deliberately coarse step makes Euler's outward spiral visible next to RK4.
echo "==> Euler vs RK4 comparison (constant_turn, dt = 0.25 s)"
"$simulator" scenarios/constant_turn.json --quiet --integrator euler --dt 0.25 --output "$output_dir/compare/constant_turn_euler.csv"
"$simulator" scenarios/constant_turn.json --quiet --integrator rk4 --dt 0.25 --output "$output_dir/compare/constant_turn_rk4.csv"
"$python" scripts/visualize.py --overlay --label "Euler, dt=0.25 s" --label "RK4, dt=0.25 s" \
  "$output_dir/compare/constant_turn_euler.csv" "$output_dir/compare/constant_turn_rk4.csv" \
  --output-dir "$output_dir/plots/compare"

echo "done: CSVs in $output_dir/, plots in $output_dir/plots/"
