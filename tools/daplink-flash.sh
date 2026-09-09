#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "$0")/.." && pwd)"
config="$project_dir/openocd.cfg"
elf="${1:?请传入要烧录的 ELF 文件}"

# CLion 调试被强制结束时，OpenOCD 可能继续占用 DAPLink。
# 这里只结束使用本工程 openocd.cfg 的残留进程。
while IFS= read -r pid; do
    cmdline="$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)"
    if [[ "$cmdline" == *"$config"* ]]; then
        kill "$pid" 2>/dev/null || true
    fi
done < <(pgrep -x openocd || true)

# 给旧进程短暂时间释放 CMSIS-DAP USB 接口。
for _ in 1 2 3 4 5 6 7 8 9 10; do
    busy=0
    while IFS= read -r pid; do
        cmdline="$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)"
        [[ "$cmdline" == *"$config"* ]] && busy=1
    done < <(pgrep -x openocd || true)
    [[ "$busy" == 0 ]] && break
    sleep 0.1
done

exec /usr/bin/openocd \
    -f "$config" \
    -c "program {$elf} verify reset exit"
