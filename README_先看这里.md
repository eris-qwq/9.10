# machine_debug_final 移植到 DM-MC02 / STM32H723

> 本文件记录早期单CAN/双步进移植。当前新硬件方案请以根目录
> `README_双CAN新版本.md` 为准，不要按本文件中的旧CAN和步进接线表接线。

这是以学长 `machine_debug_final (原)` 为业务基准、以达妙 MC02 官方
`CtrBoard-H7_ALL` 为底层骨架的移植工程。当前源码已在本机用 GNU Arm GCC 完整
编译通过，并已生成可烧录固件。

## 编译结论

你的电脑没有 Keil，但已经装有可用工具：

- STM32CubeIDE 2.2.0 自带 GNU Arm GCC 14.3.1；
- CMake；
- Ninja；
- STM32CubeIDE 自带 STM32_Programmer_CLI。

因此本工程的正式构建入口是根目录的 `CMakeLists.txt`，不是
`MDK-ARM/CtrBoard-H7_ALL.uvprojx`。后者是 MC02 官方 Keil 工程参考，未作为本次
Linux 编译入口。

> 注意：根目录的 `.ioc` 是 MC02 官方全外设例程留下的参考配置，不是本次
> 移植的唯一配置源。不要直接在 CubeMX 中点击“重新生成代码”，否则可能把
> PE2/PE7/PE8、TIM1/TIM2、CAN 过滤器和已裁剪的外设初始化覆盖回官方默认值。

在工程根目录双击/运行 `编译固件.sh`，或执行：

```bash
cmake -S . -B build-release -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-release --parallel
```

输出位于 `Firmware/`：

- `machine_debug_final_DM_MC02_H7.elf`：调试信息和符号；
- `machine_debug_final_DM_MC02_H7.hex`：推荐用 CubeProgrammer 烧录；
- `machine_debug_final_DM_MC02_H7.bin`：裸二进制，起始地址必须是 `0x08000000`。

## 已保留的业务逻辑

以下内容继续使用学长工程的文件名、变量名、类型和控制流程：

- `MDK-ARM/PID_task.c/.h` 中约 2 ms 的 `PID_Task`；
- `Remote_deal` 自动状态流程、`stage/start/once/Sequence` 等变量；
- 四轮运动解算、M2006 速度 PID、GM6020 位置/速度串级 PID；
- `MDK-ARM/Auto.c/.h` 的两套步进机构与舵机动作；
- `Robot` 中的 PID、电机反馈解析、底盘解算等业务库。

`MDK-ARM` 和 `Robot` 中保留了逐函数、关键语句的中文注释；新增的 CAN、USB、
BMI088、按钮与任务接入函数也写有中文注释。ST 官方 HAL、CMSIS、FreeRTOS 和 USB
库属于第三方底层，保留官方注释，不对库中每个函数重复改写。

## 约定的唯一业务例外

不做遥控器：UART4/DBUS 接收被移除。原自动流程仍由 `Remote_deal` 执行，但启动源
改成 PE2 常开复位按钮。按钮按下时把 PE2 与 GND 短接，约 20 ms 消抖后把原变量
`start` 置为 `2`。

为避免原 `Updatakey()` 解引用空指针，`Remote_Control` 及按键类型仍保留，但它指向
一个全零按键对象，不再对应任何遥控硬件。

## 上电前必须确认

本工程完成的是源码移植、静态核对和交叉编译，尚未代替实车完成电机悬空测试、IMU
方向标定及机械零位标定。第一次上电必须抬起底盘轮、限制电源电流，先确认：

1. 五个电调反馈 ID 与安装位置一致；
2. 车体顺/逆时针旋转时 `Now_Yaw`、`Now_Omega` 的正负号能让 Yaw PID形成负反馈；
3. 两个步进驱动器的 PUL/DIR 输入能可靠识别 3.3 V，不能识别时加 3.3 V→5 V
   缓冲/光耦驱动板；
4. 两套步进机构没有限位开关，必须人工放到约定零位后再上电；掉步或中途断电后
   `StepCounter` 不再代表真实位置；
5. PE2 只是启动按钮，不是急停；整车必须另有能切断执行器电源的实体急停/总开关。

详细引脚和核对步骤见 `文档/硬件接线与上电检查.md`，业务差异见
`文档/移植差异与代码导航.md`。
