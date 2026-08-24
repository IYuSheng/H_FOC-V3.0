# H_FOC-V3.0

基于 STM32G431CB 的步进电机 FOC 电机控制工程，支持 Linux 命令行构建（CMake + Make + ARM GNU 工具链），也可使用 Keil MDK 或 STM32CubeMX 打开源码。

## 目录结构

```
H_FOC-V3.0/
├── CMakeLists.txt               # CMake 构建脚本（交叉编译 STM32G431xx）
├── Makefile                     # 顶层构建入口：make H_FOC / make clean / make flash
├── Code/                        # 工程源码（CubeMX 工程根目录）
│   ├── Code.ioc                 # CubeMX 工程文件（可用 CubeMX 重新生成外设代码）
│   ├── Core/                    # 内核/外设初始化（CubeMX 生成 + 少量手改）
│   │   ├── Inc/                 #   adc.h, tim.h, fdcan.h, main.h, config.h ...
│   │   └── Src/                 #   main.c, adc.c, tim.c, fdcan.c, usart.c ...
│   ├── APP/                     # 应用层：FOC 控制、编码器、界面、参数配置
│   ├── MID/                     # 中间层：滤波、轨迹规划、通信、参数辨识算法
│   ├── Drivers/                 # STM32G4xx HAL 库 + CMSIS（官方驱动，勿改动）
│   ├── MDK-ARM/                 # Keil MDK 工程（可选，与 CMake 构建二选一）
│   ├── STM32G431CB_FLASH.ld     # GCC 链接脚本（128KB Flash / 32KB RAM）
│   └── build/                   # Keil 编译输出（自动生成，无需提交）
└── build/                       # CMake 编译输出（自动生成，无需提交）
```

## 环境要求

- Linux（本文以 Ubuntu/Debian 为例）
- `arm-none-eabi-gcc`（ARM GNU 工具链，本项目使用 14.3.Rel1）
- `cmake` ≥ 3.20
- `make`
- （可选）`openocd`，用于 `make flash` 烧录

## 安装编译链

### 1. 安装基础工具

```bash
sudo apt update
sudo apt install -y build-essential cmake make
```

### 2. 安装 ARM GNU 工具链

方式一：从 Arm 官网下载（推荐，版本与开发环境一致）：

```bash
# 下载 14.3.Rel1 版本（x86_64 Linux）
wget https://developer.arm.com/-/media/Files/downloads/gnu/14.3.rel1/binrel/arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi.tar.xz

# 解压到用户目录（例如 ~/app）
mkdir -p ~/app
tar -xf arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi.tar.xz -C ~/app

# 将工具链加入 PATH（可写入 ~/.bashrc 永久生效）
export PATH=$HOME/app/arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi/bin:$PATH
```

方式二：Ubuntu 软件源安装（版本可能较旧，不保证与工程完全兼容）：

```bash
sudo apt install -y gcc-arm-none-eabi
```

验证安装：

```bash
arm-none-eabi-gcc --version
# 输出示例：arm-none-eabi-gcc (Arm GNU Toolchain 14.3.Rel1) 14.3.1
```

## 构建

在工程根目录执行：

```bash
make H_FOC        # 等价于 make；首次会自动执行 cmake 配置
```

构建产物位于 `build/` 目录：

| 文件 | 说明 |
| --- | --- |
| `build/H_FOC_V3.0.elf` | ELF 可执行文件 |
| `build/H_FOC_V3.0.hex` | Intel HEX（烧录用） |
| `build/H_FOC_V3.0.bin` | 纯二进制 |
| `build/H_FOC_V3.0.map` | 链接映射（查内存占用） |

其他常用命令：

```bash
make clean        # 清理编译产物
make cmake        # 强制重新执行 cmake 配置
```

> 提示：新增/删除 `.c` 源文件后，`make` 会自动重新运行 cmake 刷新文件列表；若遇到找不到新文件的奇怪问题，先执行 `make cmake`。

## 烧录

### 方式一：OpenOCD + ST-Link（推荐命令行）

```bash
sudo apt install -y openocd
# 接线：ST-Link SWD 连接目标板，然后执行
make flash
```

### 方式二：STM32CubeProgrammer

从 ST 官网安装 [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) 后：

```bash
STM32_Programmer_CLI -c port=SWD -w build/H_FOC_V3.0.hex -v -rst
```

### 方式三：Keil MDK（Windows）

直接打开 `Code/MDK-ARM/Code.uvprojx` 编译下载，与 CMake 构建互不影响。

## 常见问题

- **`arm-none-eabi-gcc: command not found`**：工具链未加入 `PATH`，重新执行上面的 `export PATH=...` 并确认路径正确。
- **`cmake: command not found` 或版本过低**：`sudo apt install -y cmake`，或从 [cmake.org](https://cmake.org/download/) 安装新版。
- **链接警告 `_write is not implemented and will always fail`**：来自 newlib-nano 的默认桩函数，裸机工程中未实际调用，可忽略。
- **`make flash` 失败**：确认已安装 openocd、ST-Link 已连接且目标芯片为 STM32G431 系列。
