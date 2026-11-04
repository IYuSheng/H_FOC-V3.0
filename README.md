# H_MC V3.0

基于 `STM32G431CBT6` 的四相步进电机控制工程，使用 CMake、Make 和 ARM GNU
工具链构建。

## 环境准备

### Board 配置工具

STM32CubeMX，建议版本 `6.16.1`。仅用于重新生成外设配置，构建系统不依赖它，
初次使用无需安装。

CubeMX 工程文件位于 `boards/stm32g431/H_MC_V3.0.ioc`，目标工具链已经配置为
CMake。

### 依赖软件

```bash
sudo apt update
sudo apt install -y build-essential cmake make openocd
```

- CMake 版本要求不低于 `3.20`。
- `openocd` 仅在烧录时需要。

### 下载工具链和配置环境变量

工具链为 `arm-gnu-toolchain-14.3.rel1`。下方地址为 ARM 官方下载页面，也可以从
内网镜像下载同名版本。

```bash
wget https://developer.arm.com/-/media/Files/downloads/gnu/14.3.rel1/binrel/arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi.tar.xz

mkdir -p "$HOME/app"
tar -xf arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi.tar.xz -C "$HOME/app"

# 写入环境变量，注意路径
vim ~/.bashrc
export ARM_TOOLCHAIN_DIR="$HOME/app/arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi"
if [ -d "$ARM_TOOLCHAIN_DIR" ]; then
  export PATH="$ARM_TOOLCHAIN_DIR/bin:$PATH"
fi

source ~/.bashrc

# 验证
arm-none-eabi-gcc -v
```

## 编译与烧录

```bash
# 编译默认目标
make

# 等价于 make
make H_MC

# 重新执行 CMake 配置
make cmake

# 运行主机侧测试
make test

# 使用 ST-Link + OpenOCD 烧录
make flash

# 清理构建产物
make clean
```

默认构建目标为 `H_MC_V3.0`，产物目录为 `build/`。烧录前需要连接 ST-Link SWD，
并确认目标芯片为 `STM32G431` 系列。

## 产物说明

- `build/H_MC_V3.0.elf`：带调试信息的 ELF 固件，用于 OpenOCD 烧录和调试。
- `build/H_MC_V3.0.hex`：Intel HEX 固件，可用于 STM32CubeProgrammer 等工具。
- `build/H_MC_V3.0.bin`：原始二进制固件。
- `build/H_MC_V3.0.map`：链接映射文件，用于检查 Flash、RAM 占用和符号地址。

使用 STM32CubeProgrammer 烧录 HEX：

```bash
STM32_Programmer_CLI -c port=SWD -w build/H_MC_V3.0.hex -v -rst
```

## 硬件版本说明

### H_MC V3.0

MCU 为 `STM32G431CBT6`，电流采样使用 `INA240A1D` 和 `6mΩ` 采样电阻，栅极驱动
使用 `UCC27524DGNR`，功率 MOSFET 为 `WSD3045DN33`，磁编码器为
`MT6816CT-ACD-R`。

硬件资料位于 `boards/stm32g431/hardware/`：

- `schematic.pdf`：原理图。
- `PCB_top.pdf`：PCB 顶层。
- `PCB_bottom.pdf`：PCB 底层。
- `STM32G431CBT6_datasheet.pdf`：MCU 数据手册。

重新打样或更换功率器件前，需要核对栅极驱动、电流采样增益、采样电阻和功率级耐压，
并同步修改 `board_config.h` 中的硬件参数。
