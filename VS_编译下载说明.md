# Visual Studio 一键编译并下载

用 Visual Studio 2022 打开本目录（**打开本地文件夹**，不是旧的 Keil 工程），配置选择 `windows-release-flash`。按 `Ctrl+B` 会编译，再通过 ST-LINK/SWD 自动下载并复位 STM32H723。

如果用 VS Code，打开本目录后运行“终端 → 运行生成任务 → 构建并下载 STM32”；该任务也可设为默认生成任务。

首次构建会自动下载到工程 `.tools` 目录：CMake、Ninja 与 Arm GNU Toolchain；无需手动配置编译器路径。下载操作需要互联网。

STM32CubeProgrammer 是 ST 的受许可软件，必须由用户在 ST 官网接受条款后安装。默认安装完成后无需额外配置；也可以设置环境变量 `STM32_PROGRAMMER_CLI`，其值为 `STM32_Programmer_CLI.exe` 的完整路径。

下载前请连接 ST-LINK 到开发板 SWD，并确保目标板上电。下载命令使用 `SWD 4 MHz`、写入、校验并复位；失败时不会擦除固件以外的数据。
