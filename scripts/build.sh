#!/usr/bin/env bash

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_TYPE="${BUILD_TYPE:-Debug}"
build_type_lower="$(printf '%s' "${BUILD_TYPE}" | tr '[:upper:]' '[:lower:]')"
if [[ "${BUILD_TYPE}" == "Debug" ]]; then
  DEFAULT_BUILD_NAME="build"
else
  DEFAULT_BUILD_NAME="build-${build_type_lower}"
fi
BUILD_DIR="${I2W_EXAMPLE_BUILD_DIR:-${ROOT}/${DEFAULT_BUILD_NAME}}"

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "missing required tool: $1" >&2
    exit 1
  fi
}

jobs() {
  if command -v nproc >/dev/null 2>&1; then
    nproc
    return
  fi

  getconf _NPROCESSORS_ONLN
}

require_cmd cmake

cmake -S "${ROOT}" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"

cmake --build "${BUILD_DIR}" -j"$(jobs)"
