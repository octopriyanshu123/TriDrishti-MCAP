#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_TYPE="${BUILD_TYPE:-Debug}"
build_type_lower="$(printf '%s' "${BUILD_TYPE}" | tr '[:upper:]' '[:lower:]')"
if [[ "${BUILD_TYPE}" == "Debug" ]]; then
  DEFAULT_BUILD_NAME="build"
else
  DEFAULT_BUILD_NAME="build-${build_type_lower}"
fi
I2W_EXAMPLE_BUILD_DIR="${I2W_EXAMPLE_BUILD_DIR:-${PROJECT_ROOT}/${DEFAULT_BUILD_NAME}}"

binary_path() {
  local name="$1"
  if [[ "${OSTYPE:-}" == msys* || "${OSTYPE:-}" == cygwin* || "${OS:-}" == Windows_NT ]]; then
    name="${name}.exe"
  fi
  printf '%s/%s\n' "${I2W_EXAMPLE_BUILD_DIR}" "${name}"
}

require_binary() {
  local path
  path="$(binary_path "$1")"
  if [[ ! -x "${path}" ]]; then
    echo "missing built binary: ${path}" >&2
    echo "run ./scripts/build.sh first" >&2
    exit 1
  fi
}
