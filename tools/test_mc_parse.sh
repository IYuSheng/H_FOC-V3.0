#!/usr/bin/env bash

set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$(mktemp -d)"
trap 'rm -rf "${build_dir}"' EXIT

cc="${CC:-cc}"

"${cc}" \
    -std=c11 \
    -Wall -Wextra -Werror \
    -I"${workspace_dir}/src/lib/math" \
    "${workspace_dir}/tools/test_mc_parse.c" \
    "${workspace_dir}/src/lib/math/mc_parse.c" \
    -lm \
    -o "${build_dir}/test_mc_parse"

"${build_dir}/test_mc_parse"
