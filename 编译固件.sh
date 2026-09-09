#!/usr/bin/env bash
set -euo pipefail

# 始终以脚本所在目录作为工程根目录，路径中含中文也能正常编译。
project_dir="$(cd "$(dirname "$0")" && pwd)"
build_dir="$project_dir/build-vscode"

cmake --fresh -S "$project_dir" \
      -B "$build_dir" \
      -G Ninja \
      -DCMAKE_BUILD_TYPE=Release
cmake --build "$build_dir" --parallel

echo "编译完成：$project_dir/Firmware"
