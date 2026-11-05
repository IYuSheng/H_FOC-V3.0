#!/usr/bin/env bash

set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$(mktemp -d)"
trap 'rm -rf "${build_dir}"' EXIT

cc="${CC:-cc}"

"${cc}" \
    -std=c11 \
    -Wall -Wextra -Werror \
    -I"${workspace_dir}/src/modules/input_shaper" \
    -I"${workspace_dir}/boards/stm32g431/common/board" \
    "${workspace_dir}/tools/test_mc_input_shaper.c" \
    "${workspace_dir}/src/modules/input_shaper/mc_input_shaper.c" \
    -lm \
    -o "${build_dir}/test_mc_input_shaper"

"${build_dir}/test_mc_input_shaper"
