# VehicleSim

[![CI](https://github.com/char203/vehiclesim-cpp/actions/workflows/ci.yml/badge.svg)](https://github.com/char203/vehiclesim-cpp/actions/workflows/ci.yml)

A lightweight vehicle kinematics simulator written in C++17. It integrates a
kinematic bicycle model with selectable numerical integrators (Forward Euler
and RK4), runs JSON-defined driving scenarios, and exports telemetry to CSV.

> Status: under active development. See `DEVELOPMENT_LOG.md` for progress.

## Build

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

## Docker

The `Dockerfile` provides a reproducible Ubuntu 24.04 build (base image pinned
by digest, GCC 13, warnings as errors) with three stages:

```bash
# Run the full test suite in a clean Linux environment
docker build --target test -t vehiclesim-test .
docker run --rm vehiclesim-test

# Build the minimal runtime image and run a scenario
docker build -t vehiclesim .
docker run --rm vehiclesim scenarios/lane_change.json

# Write telemetry to the host
mkdir -p output
docker run --rm -v "$PWD/output:/output" vehiclesim scenarios/lane_change.json -o /output/lane_change.csv
```

The runtime image runs as an unprivileged user (uid 10001). On Linux hosts,
add `--user "$(id -u):$(id -g)"` if that user cannot write to the mounted
directory.

## License

MIT — see [LICENSE](LICENSE).
