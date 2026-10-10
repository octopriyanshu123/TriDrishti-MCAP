#!/usr/bin/env bash

set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
require_binary mcap_logger
"$(binary_path mcap_logger)" "$@"
