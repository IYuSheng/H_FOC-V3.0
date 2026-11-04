#!/usr/bin/env bash

set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$(mktemp -d)"
trap 'rm -rf "${build_dir}"' EXIT

cc="${CC:-cc}"

"${cc}" \
    -std=c11 \
    -Wall -Wextra -Werror \
    -DPRINTF_DISABLE_SUPPORT_EXPONENTIAL \
    -DPRINTF_DISABLE_SUPPORT_LONG_LONG \
    -I"${workspace_dir}/src/lib/printf" \
    "${workspace_dir}/tools/test_printf.c" \
    "${workspace_dir}/src/lib/printf/printf.c" \
    -o "${build_dir}/test_printf"

"${build_dir}/test_printf"
