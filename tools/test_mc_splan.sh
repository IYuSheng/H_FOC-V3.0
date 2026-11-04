#!/usr/bin/env bash

set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$(mktemp -d)"
trap 'rm -rf "${build_dir}"' EXIT

cc="${CC:-cc}"

"${cc}" \
    -std=c11 \
    -O2 \
    -Wall -Wextra -Werror \
    -I"${workspace_dir}/src/modules/motion_planning" \
    "${workspace_dir}/tools/test_mc_splan.c" \
    "${workspace_dir}/src/modules/motion_planning/mc_s_curve_planner.c" \
    -lm \
    -o "${build_dir}/test_mc_splan"

"${build_dir}/test_mc_splan"
