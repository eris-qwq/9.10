# 仅伸缩换 M2006：先看这里

这个文件夹是独立新版本：升降机构仍使用原步进电机，伸缩机构改为 C610 + M2006（36:1）。学长的自动流程、函数名和 `StepCounter` 逻辑位置名称继续保留。

## 当前硬件分配

| 总线/引脚 | ID | 设备 |
|---|---:|---|
| CAN1 | 0x201～0x204 | 四个底盘 M2006 |
| CAN2 | 0x202 | 伸缩 C610 + M2006（36:1） |
| CAN2 | 0x205 | GM6020 |
| PA0 + PE7 | — | 升降步进 STEP + DIR |
| Type-C | — | Linux上位机USB CDC，通常为 `/dev/ttyACM0` |

CAN2 的 0x201 控制槽暂时写0，留给以后升降也更换 M2006 时使用。

## 可调参数

集中查看 [`MDK-ARM/mechanism_config.h`](MDK-ARM/mechanism_config.h)：

- `MECH_LENGTH_MOTOR_SIGN`：伸缩方向反了，在 `1.0f` 与 `-1.0f` 之间切换。
- `MECH_LENGTH_DEG_PER_LEGACY_UNIT`：M2006转子角度与原 `StepCounter[1]` 单位的换算比例；当前 `1.0f` 只是占位值。
- `MECH_LENGTH_MIN_UNIT/MAX_UNIT`：伸缩软件行程。
- `MECH_LENGTH_POSITION_KP/MAX_SPEED_RPM`：位置响应与最高转速。
- `MECH_SPEED_KP/KI/KD`、`MECH_CURRENT_LIMIT`：C610速度环和电流上限。

没有完成比例、方向和行程标定前，不要运行整套自动流程。第一次请降低电流和速度，只做小距离测试。

其他需要调整的位置集中列在 [`文档/调参总表.md`](文档/调参总表.md)，不需要在整个工程中盲目搜索。

升降步进仍按 PA0 输出脉冲、PE7控制方向；原 `__Up__()`、`__Down__()` 调用方式未改。伸缩的 `__Length__()`、`__Shorten__()` 内部已变为 M2006 位置闭环。

## 断电位置与原点开关

程序会把升降 `StepCounter[0]` 和伸缩 `StepCounter[1]` 保存到板载W25Q64，下次上电继续使用。它只能保存软件估计位置；断电后人工移动、打滑或中途掉电都会产生误差。

以后加原点开关时，每次开机先低速碰到固定开关，再调用 `Mechanism_ApplyOrigin()` 把该点定义为0。当前没有分配GPIO，也不会读取未接的开关，详见 [`文档/原点开关_以后接入说明.md`](文档/原点开关_以后接入说明.md)。

## 编译

```bash
./编译固件.sh
```

本版本已经编译通过。固件位于 `Firmware/machine_debug_DM_MC02_length_M2006.hex`，当前没有自动烧录开发板。
