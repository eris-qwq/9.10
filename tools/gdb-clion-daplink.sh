#!/usr/bin/env bash
set -euo pipefail

# CLion通过MI协议发送 -exec-interrupt。显式启用异步模式，使目标运行时
# GDB仍能处理暂停、停止和断点事件。
exec /usr/bin/gdb-multiarch \
    -ex "set mi-async on" \
    "$@"
