# syntax=docker/dockerfile:1
#
# Reproducible Linux build of VehicleSim.
#
# Stages:
#   build    compiler toolchain + Release build (warnings are errors)
#   test     runs the full CTest suite:   docker build --target test -t vehiclesim-test .
#                                         docker run --rm vehiclesim-test
#   runtime  minimal image with the CLI:  docker build -t vehiclesim .
#                                         docker run --rm vehiclesim scenarios/lane_change.json
#
# The base image is pinned by its multi-arch index digest (amd64 and arm64),
# so rebuilding later starts from byte-identical layers. Ubuntu packages
# still come from the live 24.04 archive.

ARG BASE_IMAGE=ubuntu:24.04@sha256:68ee22004a9109b44973313b02ff7b1de250f1de0f5ecb628d73f579aaff33ac

# ---------------------------------------------------------------------------
FROM ${BASE_IMAGE} AS build

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        ninja-build \
        python3 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src
# Copy only what the build needs so unrelated edits (docs, README) do not
# invalidate the build cache.
COPY CMakeLists.txt ./
COPY cmake/ cmake/
COPY include/ include/
COPY src/ src/
COPY tests/ tests/
COPY scenarios/ scenarios/
COPY scripts/validate_csv.py scripts/

RUN cmake -S . -B build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DVEHICLESIM_WARNINGS_AS_ERRORS=ON \
    && cmake --build build

# ---------------------------------------------------------------------------
FROM build AS test
CMD ["ctest", "--test-dir", "build", "--output-on-failure"]

# ---------------------------------------------------------------------------
FROM ${BASE_IMAGE} AS runtime

# libstdc++ is already part of the Ubuntu base image; nothing else is needed.
RUN useradd --create-home --uid 10001 simulator
COPY --from=build /src/build/vehiclesim /usr/local/bin/vehiclesim
COPY --chown=simulator:simulator scenarios/ /opt/vehiclesim/scenarios/

USER simulator
WORKDIR /opt/vehiclesim
ENTRYPOINT ["vehiclesim"]
CMD ["--help"]
