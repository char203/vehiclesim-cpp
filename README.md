# VehicleSim

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

## License

MIT — see [LICENSE](LICENSE).
