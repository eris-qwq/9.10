# 上位机改用 MC02 Type-C USB CDC

MC02 固件枚举为 STM32 USB CDC（VID `0483`、PID `5740`）。学长上位机已有对应
udev 规则，但 `src/main.cpp` 仍写成了 CH340 的 `ttyUSB15`，所以只需要这一处改成
`ttyACM15`。

本目录的 `替换文件/src/main.cpp` 已完成修改；把它覆盖到学长上位机
`NUTEIA_v5/src/main.cpp`。`替换文件/99-NUTEIA-fix.rules` 是学长原有规则，里面
已经包含：

```text
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="ttyACM15", MODE="0666"
```

首次在 Linux 上安装规则：

```bash
sudo cp 99-NUTEIA-fix.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

重新插拔 MC02 Type-C 后检查：

```bash
ls -l /dev/ttyACM15
```

注意：USB CDC 是虚拟串口，上位机中保留 `115200` 不会限制 USB 实际吞吐量，但可
继续作为串口配置参数使用。
