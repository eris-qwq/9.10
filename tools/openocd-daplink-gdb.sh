#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "$0")/.." && pwd)"
config="$project_dir/openocd.cfg"

# CLion异常终止时偶尔会留下OpenOCD。只清理使用本工程配置的残留进程，
# 不影响其他工程或调试器。
while IFS= read -r pid; do
    [[ "$pid" == "$$" ]] && continue
    cmdline="$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)"
    if [[ "$cmdline" == *"$config"* ]]; then
        kill "$pid" 2>/dev/null || true
    fi
done < <(pgrep -x openocd || true)

# CLion只需要GDB的3333端口；禁用Tcl/Telnet端口可避免无关冲突。
exec /usr/bin/openocd \
    -c "tcl_port disabled" \
    -c "telnet_port disabled" \
    -f "$config"
