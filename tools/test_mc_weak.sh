#!/usr/bin/env bash

set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$(mktemp -d)"
trap 'rm -rf "${build_dir}"' EXIT

cc="${CC:-cc}"

"${cc}" \
    -std=c11 \
    -Wall -Wextra -Werror \
    -I"${workspace_dir}/src/modules/motor_control" \
    "${workspace_dir}/tools/test_mc_weak.c" \
    "${workspace_dir}/src/modules/motor_control/mc_weak.c" \
    -lm \
    -o "${build_dir}/test_mc_weak"

"${build_dir}/test_mc_weak"
echo "mc_weak: PASS"
