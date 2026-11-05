#!/usr/bin/env bash

set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$(mktemp -d)"
trap 'rm -rf "${build_dir}"' EXIT

cc="${CC:-cc}"

"${cc}" \
    -std=c11 \
    -Wall -Wextra -Werror \
    -I"${workspace_dir}/src/lib/vofa" \
    -I"${workspace_dir}/boards/stm32g431/common/hal" \
    -I"${workspace_dir}/boards/stm32g431/common/board" \
    "${workspace_dir}/tools/test_vofa.c" \
    "${workspace_dir}/src/lib/vofa/vofa_plot.c" \
    -lm \
    -o "${build_dir}/test_vofa"

"${build_dir}/test_vofa"
