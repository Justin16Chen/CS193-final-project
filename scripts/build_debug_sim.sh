#!/usr/bin/env bash
# Build with AddressSanitizer + UBSan. Same as: build_sim.sh --asan
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$SCRIPT_DIR/build_sim.sh" --asan "$@"
