# EndlessFlight H7Ultra 自定义 PX4/LQI 固件移植与修改记录
**文档用途**：记录 Ryan 自定义 PX4/LQI 固件向 EndlessFlight H7Ultra（STM32H743VIT6）移植的历史提交、后续本地配置更新、构建与联调证据；用于仓库归档、项目交接和后续复现。

**更新日期**：2026-11-16。

**历史差异基线**：`Ryan-5853/PX4-TeleaiTrans` 提交 `3fef8242687cf733b590ff21c5b36779943299d1` → 历史移植提交 `12d74dbb53`（原文统计的 37 个文件只适用于这两个固定版本）。

**目前移植分支**：`Ryan-5853/PX4-TeleaiTrans` 的 `teleai/v1.16.2-endlessflight-h7ultra`；此前确认远程 HEAD 为 `c521c9e990b99ec07ff116ea7799a25befe81b14`。本文 2026-11-16 新增的 5 文件改动来自**用户本地 `git diff`，尚未作为远程提交验证**。

**上游同步范围**：本文新增内容不代表已经合并 Ryan 2026-11-12 的 `3dad7184942992325173f970b1b889ee7eecb414`。原有 LQI 模型/算法版本仍以实际工作区和固件为准。

**验证边界**：历史固件的编译、USB/GS-Link 联调有运行记录；**2026-11-16 本地串口修改已通过 `git diff --check`，没有提供新一轮编译成功、刷写或 UART3 实测结果**。严禁将源码修改直接写成飞行安全或实机验证通过。

## 1. 项目背景与移植目标
原始固件基于 PX4 v1.16.2 的 Ryan 自定义控制框架，主要面向 MicoAir H743 V2 飞控，并集成共轴双电机三舵机 LQI 控制器、`nn_control`、`gs_link` 和 `airlink` 等功能。

本次目标是：**在不修改 Ryan 原有 LQI 控制算法和通信协议实现的前提下，使对应自定义固件能够在 EndlessFlight H7Ultra（STM32H743VIT6）上编译、启动、接受 ELRS/CRSF 遥控输入，并通过 ESP32-C5 与 Dock Ground Station 进行 GS-Link 通信。**

当前构建目标（在 VS Code 的 Ubuntu WSL2 终端中执行）：

```bash
cd ~/projects/PX4-H7Ultra-Ryan
source ~/venvs/px4-h7ultra/bin/activate  # 若虚拟环境位于该路径
make endlessflight_h743ultra_nncontrol
```
本次建议使用第 9.2 节的**带时间戳构建归档流程**，同时保存日志、`*.px4` 固件、源码差异及 SHA-256。

本次主要成果可归纳为：

1. **硬件 BSP 移植**：引入 H7Ultra 的 NuttX、GPIO、UART、SPI/I2C、DMA、PWM/DShot、SDIO、ADC、USB、Bootloader 和固件打包配置。

2. **公共驱动扩展**：增加 QMC5883P 磁力计驱动及设备类型、构建系统注册。

3. **自定义固件构建**：新增 H7Ultra 的 `nncontrol` / `gslink` 等构建目标，沿用原控制框架。

4. **启动与参数适配**：预置 RC/CRSF/GPS/机架参数、执行器输出相关默认值及 LQI 台架安全门控。

5. **通信部署改动（历史）**：将 GS-Link 自动启动波特率由 4 Mbps 调为 5 Mbps；旧固件已有双向通信验证记录。
6. **通信配置增量（2026-11-16，未提交/未实机复测）**：统一 GS-Link 参数、模块默认值和 QGC 波特率选项为 5 Mbps；H7Ultra 将 Airlink 从 UART2 迁移至 UART3，GPS 暂不启用。

## 2. 全仓库 Git 差异统计
### 2.1 历史提交差异（冻结口径）

从 `3fef824` 到 `12d74db`，经 Git 提交比较：

| 指标 | 数量 |
|---|---:|
| 差异文件总数 | **37** |
| 新增路径 | **34** |
| 修改已有路径 | **3** |
| 直接修改 LQI 核心代码 | **0** |
| 直接修改 GS-Link 协议实现 | **0** |
| 直接修改 AirLink 协议实现 | **0** |

按路径统计：

| 范围 | 文件数 | 用途 |
|---|---:|---|
| `boards/endlessflight/h743ultra/` | 26 | H7Ultra BSP、NuttX、固件目标、启动脚本、Bootloader |
| `src/drivers/` | 8 | QMC5883P 驱动及公共注册 |
| `Tools/h7ultra_port/` | 2 | BSP 导入工具和来源说明 |
| `ROMFS/px4fmu_common/init.d/rc.direct_control_apps` | 1 | GS-Link 启动波特率 |

这份统计对**整个移植分支**成立，并非只统计最后一次 Git 提交。

### 2.2 历史移植时修改的原有三个文件
| 原有文件 | 变更内容 |
|---|---|
| `ROMFS/px4fmu_common/init.d/rc.direct_control_apps` | `gs_link` 启动参数由 `-b 4000000` 改为 `-b 5000000` |
| `src/drivers/drv_sensor.h` | 增加 QMC5883P 的磁力计设备类型标识 |
| `src/drivers/magnetometer/CMakeLists.txt` | 将 QMC5883P 纳入磁力计驱动构建 |

其余 34 个文件相对于原始 Ryan 基线都是**新增路径**。其中部分文件在移植过程中后来又进行了二次修改，但在全分支比较中依然计为新增。

### 2.3 2026-11-16 本地工作区新增改动（独立统计）

来自用户在 `~/projects/PX4-H7Ultra-Ryan` 执行的 `git diff --stat` 和 `git diff --check`：

```text
5 files changed, 21 insertions(+), 3 deletions(-)
```
| 路径 | 本次增量 | 目的 |
|---|---|---|
| `ROMFS/px4fmu_common/init.d/airframes/22001_direct_actuator_controller` | `SER_TEL1_BAUD: 4000000 → 5000000` | 保持机架参数显示与 GS-Link 部署速率一致 |
| `src/modules/gs_link/GsLink.cpp` | 默认波特率 `4000000 → 5000000` | 避免手动不带 `-b` 启动时退回 4 Mbps |
| `Tools/serial/serial_params.c.jinja` | 添加 `@value 5000000` | 在通用串口参数的枚举选项中加入 5 Mbps |
| `ROMFS/px4fmu_common/init.d/rc.direct_control_apps` | Airlink 使用 `${AIRLINK_PORT}`，默认回退 `/dev/ttyS1` | 支持不同板型使用不同 Airlink 串口 |
| `boards/endlessflight/h743ultra/init/rc.board_defaults` | 设置 `AIRLINK_PORT=/dev/ttyS2`；条件清除占用 TELEM2 的 MAVLink 实例 | 将 H7Ultra Airlink 迁至 UART3 |

**统计说明**：这 5 个文件**不是原历史 37 个文件的重新统计**，而是相对当前工作树 HEAD 的一组未提交增量；二者不能直接相加得出“新版全仓库差异”。其中 `GsLink.cpp` 只修改了启动默认参数，**未改报文协议、CRC 和状态机**。

**已检查**：`git diff --check` 无报错；所提供差异符合上述 5 文件清单。**尚未检查**：新代码实际编译、固件打包、UART3 硬件连接和传输稳定性。

## 3. 硬件 BSP 移植（主要工作）
### 3.1 新增板级文件与构建支持
新增目录：`boards/endlessflight/h743ultra/`。

主要内容包括：

| 类别 | 文件/目录 | 作用 |
|---|---|---|
| 固件目标 | `default.px4board`、`gslink.px4board`、`nncontrol.px4board`、`bootloader.px4board` | 定义编译进入固件的驱动、控制模块和工具 |
| 固件打包 | `firmware.prototype` | H7Ultra 固件元信息与镜像限制 |
| Bootloader 文件 | `extras/endlessflight_h743ultra_bootloader.bin` | 板级引导相关二进制文件；提交到源码不意味着此次已刷写 Bootloader |
| NuttX | `nuttx-config/bootloader/defconfig`、`nuttx-config/nsh/defconfig` | 系统和 Bootloader 编译配置 |
| 内存与 DMA | `nuttx-config/include/board.h`、`board_dma_map.h`、`scripts/*.ld` | 引脚、DMA、链接地址与内存布局 |
| 板级源码 | `src/board_config.h`、`hw_config.h`、`init.c`、`CMakeLists.txt` | MCU 外设定义与初始化 |
| I/O | `src/i2c.cpp`、`spi.cpp`、`timer_config.cpp` | 外设总线、执行器定时器和输出映射 |
| 其他板级功能 | `src/sdio.c`、`usb.c`、`led.c`、`bootloader_main.c` | SD 卡、USB、指示灯和引导入口 |

**来源说明**：这部分主要基于厂家 H7Ultra PX4 1.16.0 BSP 导入，再适配 Ryan 的 PX4 v1.16.2。不能将其全部表述为原创编写的底层驱动代码。

### 3.2 与 MicoAir 的主要硬件差异
| 子系统 | 移植后 H7Ultra | Ryan 原有 MicoAir 平台 |
|---|---|---|
| 板级路径 | `boards/endlessflight/h743ultra` | `boards/micoair/h743-v2` |
| IMU | SCH16T + 双 IIM42652 | BMI088 + BMI270 |
| 气压计 | SPA06 | SPL06 |
| 磁力计 | QMC5883P | QMC5883L |
| GS-Link 串口设备 | `/dev/ttyS0` | `/dev/ttyS0` |
| AirLink 串口设备 | **`/dev/ttyS2`（2026-11-16 本地方案，UART3/TELEM2）** | `/dev/ttyS1`（历史原方案） |
| 原生 RC 串口设备 | `/dev/ttyS4` | `/dev/ttyS5` |
| PWM/DShot | H7Ultra GPIO、定时器、DMA 映射 | MicoAir 对应的独立映射 |

**注意**：串口设备名相同不等于物理引脚相同。真正的 UART/GPIO/定时器关系由各自 BSP 定义。

H7Ultra 当前串口规划（**源码期望值，非新固件实测**）：

| H7Ultra 接口 | 设备 | 计划用途 | 波特率 |
|---|---|---|---:|
| UART1 / TELEM1 | `/dev/ttyS0` | GS-Link ↔ ESP32-C5 | 5,000,000 |
| UART2 / GPS1 | `/dev/ttyS1` | 预留 GPS1，当前关闭 | 不启动 |
| UART3 / TELEM2 | `/dev/ttyS2` | Airlink | 6,000,000 |
| UART4 / TELEM3 | `/dev/ttyS3` | 预留 | — |
| UART5 / RC | `/dev/ttyS4` | ELRS / CRSF | 按 RC 配置 |
| UART6 / GPS2 | `/dev/ttyS5` | 预留 GPS2，当前关闭 | 不启动 |
| UART7 | `/dev/ttyS6` | 预留 | — |
| UART8 / TELEM4 | `/dev/ttyS7` | 预留 | — |

此处的“UART3”是**飞控板物理 UART3/TELEM2**，不是 ESP32-C5 的 UART3。

## 4. BSP 之外的源码改动
### 4.1 新增 QMC5883P 磁力计驱动
原始 Ryan 基线没有供 H7Ultra 使用的这一套 QMC5883P 驱动，本次新增：

```text
src/drivers/magnetometer/qmc5883p/
├── CMakeLists.txt
├── Kconfig
├── QMC5883P.cpp
├── QMC5883P.hpp
├── QST_QMC5883P_registers.hpp
└── qmc5883p_main.cpp
```
另修改公共 `drv_sensor.h` 与磁力计目录 `CMakeLists.txt`，使新驱动可被构建系统识别。

H7Ultra 传感器启动脚本中的主要命令：

```sh
board_adc start
sch16t -s -b 1 -R 2 start
iim42652 -s -b 2 -R 4 start
iim42652 -s -b 3 -R 6 start
spa06 -I -a 0x76 start
qmc5883p -I -a 0x2C -R 6 start
```
驱动编译成功与飞控能够启动不代表全部传感器校准、安装方向及数据精度已经完成飞行级验证。

### 4.2 创建 H7Ultra 专用 `nncontrol` 编译目标
新增：`boards/endlessflight/h743ultra/nncontrol.px4board`。

保留核心自定义模块：

- `CONFIG_MODULES_NN_CONTROL=y`

- `CONFIG_MODULES_GS_LINK=y`

- `CONFIG_MODULES_AIRLINK=y`

- `CONFIG_MODULES_COMP_ATTITUDE=y`

- `CONFIG_NN_CONTROL_BACKEND_LQR=y`

同时关闭普通 PX4 多旋翼/固定翼飞控链路中的若干模块（例如 `MC_ATT_CONTROL`、`MC_RATE_CONTROL`、`MC_POS_CONTROL`、`CONTROL_ALLOCATOR`、`FLIGHT_MODE_MANAGER`），保留 Commander、MAVLink、遥控输入、EKF2、传感器、Logger 等必要功能。由于是精简目标，QGC 的部分标准功能页面可能引用未编入固件的参数（如 `CP_DIST`）。

**本项属于固件构建与软件模块集成，不是对 LQI 算法的修改。**

### 4.3 机架与 RC/GPS 出厂默认值
文件：`boards/endlessflight/h743ultra/init/rc.board_defaults`。

专用 `22001` 机架存在时，预置：

```sh
param set-default SYS_AUTOSTART 22001
param set-default RC_PORT_CONFIG 300
param set-default RC_SBUS_PRT_CFG 0
param set-default RC_CRSF_PRT_CFG 0
param set-default GPS_1_CONFIG 0
```
文件还具有公共预设：

```sh
param set-default RC_INPUT_PROTO -1
```
设计特点：

**2026-11-16 增量（同一文件）**：在专用 `22001` 分支增加 `set AIRLINK_PORT /dev/ttyS2`；只有检测到 `MAV_1_CONFIG` 或 `MAV_2_CONFIG` 等于 `102`（TELEM2）时，才设置为 `0`，防止这些 MAVLink 实例在启动串口配置时抢占 UART3。是否存在其他串口服务占用仍需上机检查。

- **`param set-default`** 定义出厂默认值；QGC 中已保存的用户参数不应被无条件覆盖。

- 删除了旧的 `SYS_AUTOSTART != 22001` 时强制 `param set` 回 `22001` 的逻辑。

- 未打包 `22001` 机架的普通 H7Ultra 固件仍默认 `SYS_AUTOSTART=4001`。

- 用户如果已保存其他机架编号，默认值不会强制覆盖；但切换到未打包的机架可能导致启动配置不可用。

另外保留了仅用于专用固件台架调试的强制安全保护：

```sh
param set NN_LQI_OUTPUT_EN 0
```
这行**不是**“QGC 可修改且断电后仍保持使能”的普通默认值逻辑。它在相应启动分支执行时会重新把物理输出关闭，这是刻意保留的安全行为。

### 4.4 执行器和传感器脚本中的预设
文件：`boards/endlessflight/h743ultra/init/rc.board_sensors`。专用目标新增：

```sh
param set-default PWM_MAIN_TIM0 -4
param set-default PWM_MAIN_TIM1 400
param set-default PWM_MAIN_TIM2 400
param set-default DSHOT_BIDIR_EN 1
param set-default GPS_1_CONFIG 0
```
主要用于 DShot 电机、舵机 PWM 与双向 DShot 测速相关默认配置。

**需注意默认值的重复来源**：继承的公共机架 `22001_direct_actuator_controller` 中也包含 PWM 参数预设。因此当前文件结构里存在多处默认值配置；实际启动结果还受启动顺序、已有保存参数和硬件定时器分组约束影响，应以刷写后参数检查为准。

### 4.5 GS-Link 启动波特率固定为 5 Mbps（历史验证）
直接修改了 Ryan 原有启动脚本：

文件：`ROMFS/px4fmu_common/init.d/rc.direct_control_apps`

```diff
- gs_link start -d /dev/ttyS0 -b 4000000
\+ gs_link start -d /dev/ttyS0 -b 5000000
```
该调整改变了**启动时串口驱动的波特率**，没有修改 GS-Link 报文定义、CRC、心跳或收发协议代码。`gs_link start -d /dev/ttyS0 -b 5000000` 保持不变。

已通过实机：

```text
INFO [gs_link] port: /dev/ttyS0, baud: 5000000, open: 1
```
### 4.6 2026-11-16：GS-Link 5 Mbps 一致性修复（本地代码，待编译）

此前存在三处不一致：

- `rc.direct_control_apps` 自动启动使用 `-b 5000000`（旧固件已经验证）;
- `22001_direct_actuator_controller` 中 `SER_TEL1_BAUD=4000000`;
- `GsLink.cpp::task_spawn` 在手动 `gs_link start` 不带 `-b` 时默认 `4000000`，且串口参数模板缺少 5 Mbps 枚举。

本地修正后的关键设置：

```sh
# ROMFS/px4fmu_common/init.d/airframes/22001_direct_actuator_controller
param set MAV_0_CONFIG 0
param set SER_TEL1_BAUD 5000000

# ROMFS/px4fmu_common/init.d/rc.direct_control_apps
gs_link start -d /dev/ttyS0 -b 5000000
```
```cpp
// src/modules/gs_link/GsLink.cpp (task_spawn 默认值)
uint32_t baudrate = 5000000;
```
```text
# Tools/serial/serial_params.c.jinja 补充的参数枚举行
 * @value 5000000 5000000 8N1
```
**重要区分**：GS-Link 实际串口速率由启动时 `-b`（或 `GsLink.cpp` 的默认参数）决定；`SER_TEL1_BAUD` 主要用于 PX4 通用串口参数和 QGC 展示，它**不能直接替代 GS-Link 的 `-b` 参数**。ESP32-C5 桥接 UART 也必须设置为 5 Mbps。此外，`GsLink.cpp` 默认值和通用串口参数模板是**公共代码改动**，不是 H7Ultra 板级独有的改动；其他板型如果不带 `-b` 手动启动 GS-Link，也会受到新默认值影响。

### 4.7 2026-11-16：Airlink 改用 H7Ultra UART3（本地代码，待实测）

公共启动脚本将固定设备改为可配置端口，保留其他板型的原默认行为：

```sh
# ROMFS/px4fmu_common/init.d/rc.direct_control_apps
if [ "$AIRLINK_PORT" = "" ]
then
    set AIRLINK_PORT /dev/ttyS1
fi
airlink start -d ${AIRLINK_PORT} -b 6000000
```
H7Ultra 的 `rc.board_defaults` 在专用机架配置分支增加：

```sh
# H7Ultra UART3 / TELEM2
set AIRLINK_PORT /dev/ttyS2

# 避免已有 MAVLink 参数占用 TELEM2（串口选择值 102）
if param compare MAV_1_CONFIG 102
then
    param set MAV_1_CONFIG 0
fi
if param compare MAV_2_CONFIG 102
then
    param set MAV_2_CONFIG 0
fi
```
- `gs_link` 继续独占 UART1 `/dev/ttyS0`，Airlink 改用 `/dev/ttyS2`，两者不是同一串口。
- `GPS_1_CONFIG` 在板级默认值与传感器脚本中继续为 `0`；GPS2 保留硬件映射，当前按未启用使用。**未新增** GNSS 航向融合、辅助全球位置融合、PPS 捕获或 DroneCAN GNSS。
- `/dev/ttyS2` 属于 H7Ultra 的 UART3 / TELEM2；必须检查 `MAV_0_CONFIG`、`MAV_1_CONFIG`、`MAV_2_CONFIG` 以及其他服务的实际占用。
- 现有脚本中的旧注释“`ttyS1 is UART2/GPS2`”描述不准确，**应更正为 UART2/GPS1**；是否已更正需以工作树最新 `git diff` 为准。
- 代码审阅阶段确认了此配置逻辑；**尚无 UART3 在 6 Mbps 下的 Airlink 双向通信实测记录**。

### 4.8 工程复现工具与说明
新增：

```text
Tools/h7ultra_port/
├── README.md
└── import_vendor_bsp.py
```
作用是记录上游源码与厂家 BSP 的固定来源、导入流程及复现方法。现有 README 的部分早期描述（如“尚未编译验证”）已滞后于实际完成进度，后续应更新。

## 5. 保留的 Ryan 控制算法与通信协议（启动默认值例外）
就 **LQI 核心和通信协议**而言，相对于原始 Ryan 基线，相关核心逻辑仍保持不变。注意 `GsLink.cpp` 在 2026-11-16 的本地增量中**有一次参数默认值修改**，不能再笼统称为“文件没有任何源码差异”：

| 模块 | 代表文件/范围 | 状态 |
|---|---|---|
| LQI 控制调度 | `src/modules/nn_control/NNControl.cpp` | 保留原实现 |
| LQI 核心运算 | `LqiCoaxialCollectiveCore.hpp` | 保留原实现 |
| LQI 模型 ABI | `LqiCoaxialCollectiveModel.hpp` | 保留原实现 |
| LQI 权重载荷 | `LqiCoaxialCollectivePayload.hpp` | 保留原实现 |
| GS-Link 协议 | `src/modules/gs_link/GsLinkProtocol.hpp` | 协议仍保留；`GsLink.cpp` 仅调整手动启动默认波特率（见 4.6） |
| AirLink 协议与状态机 | `src/modules/airlink/` | 保留原实现 |
| EKF2 / Commander 核心 | `src/modules/ekf2/`、`src/modules/commander/` | 保留原实现 |

因此应准确表述为：**基于现有算法完成硬件移植与部署适配；2026-11-16 仅调整通信启动与串口默认配置，并未重新开发 LQI 控制算法或通信协议。**

当前实机报告仍采用 `airframe-1` 载荷：

```text
backend: lqi_coaxial_collective
airframe: airframe-1, expected id 1, configured id 1, mismatch 0
verification weights: 504 bytes, checksum: 0x78ae1111
```
新组装飞机 `airframe-3` 的机体辨识和专用载荷部署属于后续工作，不能混同为本次已完成事项。

## 6. 历史实机验证与本次修改的验证边界
以下是**旧版已刷写固件**的运行记录所支持的结果（不等于飞行安全认证，也不自动覆盖最新本地修改）：

| 项目 | 结果 | 依据/说明 |
|---|---|---|
| H7Ultra `nncontrol` 构建 | 已成功 | 生成 `endlessflight_h743ultra_nncontrol.px4` |
| 固件刷写与 QGC USB | 已成功 | 能进入 NSH 并查询参数与运行状态 |
| 机架 | `22001` | `param show SYS_AUTOSTART` |
| RC 端口 | `300` | `param show RC_PORT_CONFIG` |
| RC 协议 | 当前保存为 `6` | 固件默认 `-1`，已保存的 CRSF 值 `6` 优先生效 |
| SBUS / CRSF 额外端口 | `0 / 0` | 参数查询确认 |
| GPS1 配置 | `0` | 历史参数查询确认；2026-11-16 本地增量继续保留禁用 |
| GS-Link 串口 | `5000000` / `open: 1` | 重新上电后的 `gs_link status` |
| GS-Link 端到端通信 | 成功验证过 | 曾出现 `heartbeat OK`、双向包数增加、CRC 错误为零 |
| `nn_control` 频率 | `500.0 Hz` | `nn_control status` |
| LQI 物理输出 | `inhibited` | `NN_LQI_OUTPUT_EN=0`，台架保护 |

GS-Link 联调链路：

```text
Dock Ground Station（Ryan 服务器）
    ↕  Wi-Fi / UDP
XIAO ESP32-C5（DroneBridge）
    ↕  UART1 / 5 Mbps
EndlessFlight H7Ultra（PX4 GS-Link + nn_control）
```
注意：有一次重启后查询时也出现过 `heartbeat stale` 和 `rx packets: 0`，这属于当次地面站通信未建立的状态，不应写成“任意时刻持续在线”。用户后续确认地面站链路可正常使用。

### 6.1 2026-11-16 本地修改验证状态

| 检查项目 | 状态 | 证据/备注 |
|---|---|---|
| 五文件源码变更 | **已检查** | 用户提供 `git diff --stat`：5 文件，21 行新增、3 行删除 |
| Git 格式检查 | **通过** | `git diff --check` 无报错 |
| GS-Link 自动启动 `-b 5000000` | **源码保留** | 先前已生效；这次没有改动启动命令 |
| GS-Link 默认波特率 / TELEM1 参数 / QGC 枚举 | **源码已修改** | 见 4.6；尚未复核新固件参数展示 |
| Airlink `/dev/ttyS2` / 6 Mbps | **源码已配置** | 见 4.7；尚未上机确认 |
| 新版固件编译 | **待提供结果** | 没有本次 `make` 退出码或日志 |
| 新版固件刷写 | **未提供验证结果** | 不能沿用历史刷写成功结论 |
| UART3 Airlink 6 Mbps 稳定性 | **未验证** | 需要实际设备和收发测试 |
| ESP32-C5 GS-Link 5 Mbps 回归测试 | **待复测** | 验证双向包数、CRC、心跳 |
| 控制输出/飞行 | **未验证** | 保持 `NN_LQI_OUTPUT_EN=0` 的台架安全配置 |

## 7. 尚未完成或需要后续复核的事项
1. **实际执行器台架验证**：确认电机/舵机通道、方向、限位、DShot RPM 与失效输出；带电测试必须保持拆桨及可靠安全隔离。

2. **启动参数一致性**：GS-Link 的 `SER_TEL1_BAUD`、模块默认值及枚举已在本地调整至 5 Mbps，仍须编译及重启复核；PWM 参数多处默认值仍需单独核对。

3. **控制载荷匹配**：目前仍使用 `airframe-1`；未来需生成、验证并部署真正的 `airframe-3` 模型载荷。

4. **磁力计和 IMU 实测校准**：检查驱动读数、坐标变换、偏置与冗余传感器选择，而不仅是能启动。

5. **固件板型识别**：H7Ultra 与原 MicoAir 的 `firmware.prototype` 均出现 `board_id=1179`；需要进一步核查 Bootloader/板型匹配，避免刷错固件。

6. **README 与发布追溯**：更新导入文档中的过时说明；使用第 9.2 节流程保存带时间戳的 `.px4`、构建日志、SHA-256、Git 状态及源码补丁。

7. **飞行安全验证**：通信和 500 Hz 运行成功不代表已经完成解锁、失控保护或首次飞行验证。在此之前继续保持输出禁用。
8. **Airlink UART3 串口验证**：在拆桨状态下确认 `airlink status` 显示 `/dev/ttyS2`、`6000000`，无 MAVLink 占用，硬件 UART3 TX/RX 和对端设置一致；检查持续收发、错误及超时。
9. **GS-Link 5 Mbps 回归验证**：确认 `gs_link status` 显示 `/dev/ttyS0`、`5000000`，ESP32-C5 同为 5 Mbps，并验证 UDP 心跳、CRC 和丢包。
10. **上游同步另行进行**：2026-11-12 的 Ryan 上游提交包含控制/通信等大量改动，尚不能算已纳入这次仅 5 文件的串口配置更新。

## 8. 完整改动文件清单（历史基线）与本地增量
### 8.1 `boards/endlessflight/h743ultra/`（26 个）
```text
boards/endlessflight/h743ultra/bootloader.px4board
boards/endlessflight/h743ultra/default.px4board
boards/endlessflight/h743ultra/extras/endlessflight_h743ultra_bootloader.bin
boards/endlessflight/h743ultra/firmware.prototype
boards/endlessflight/h743ultra/gslink.px4board
boards/endlessflight/h743ultra/init/rc.board_defaults
boards/endlessflight/h743ultra/init/rc.board_extras
boards/endlessflight/h743ultra/init/rc.board_sensors
boards/endlessflight/h743ultra/nncontrol.px4board
boards/endlessflight/h743ultra/nuttx-config/bootloader/defconfig
boards/endlessflight/h743ultra/nuttx-config/include/board.h
boards/endlessflight/h743ultra/nuttx-config/include/board_dma_map.h
boards/endlessflight/h743ultra/nuttx-config/nsh/defconfig
boards/endlessflight/h743ultra/nuttx-config/scripts/bootloader_script.ld
boards/endlessflight/h743ultra/nuttx-config/scripts/script.ld
boards/endlessflight/h743ultra/src/CMakeLists.txt
boards/endlessflight/h743ultra/src/board_config.h
boards/endlessflight/h743ultra/src/bootloader_main.c
boards/endlessflight/h743ultra/src/hw_config.h
boards/endlessflight/h743ultra/src/i2c.cpp
boards/endlessflight/h743ultra/src/init.c
boards/endlessflight/h743ultra/src/led.c
boards/endlessflight/h743ultra/src/sdio.c
boards/endlessflight/h743ultra/src/spi.cpp
boards/endlessflight/h743ultra/src/timer_config.cpp
boards/endlessflight/h743ultra/src/usb.c
```
### 8.2 公共磁力计驱动及注册（8 个）
```text
src/drivers/drv_sensor.h
src/drivers/magnetometer/CMakeLists.txt
src/drivers/magnetometer/qmc5883p/CMakeLists.txt
src/drivers/magnetometer/qmc5883p/Kconfig
src/drivers/magnetometer/qmc5883p/QMC5883P.cpp
src/drivers/magnetometer/qmc5883p/QMC5883P.hpp
src/drivers/magnetometer/qmc5883p/QST_QMC5883P_registers.hpp
src/drivers/magnetometer/qmc5883p/qmc5883p_main.cpp
```
### 8.3 移植工具（2 个）
```text
Tools/h7ultra_port/README.md
Tools/h7ultra_port/import_vendor_bsp.py
```
### 8.4 历史公共启动脚本（1 个）
```text
ROMFS/px4fmu_common/init.d/rc.direct_control_apps
```
### 8.5 2026-11-16 本地增量文件（5 个，勿计入历史文件数）

```text
ROMFS/px4fmu_common/init.d/airframes/22001_direct_actuator_controller
ROMFS/px4fmu_common/init.d/rc.direct_control_apps
Tools/serial/serial_params.c.jinja
boards/endlessflight/h743ultra/init/rc.board_defaults
src/modules/gs_link/GsLink.cpp
```
## 9. 版本引用、WSL2 复现与带时间戳的构建归档

### 9.1 历史提交与当前工作区

- Ryan 历史基线：[3fef8242687c](https://github.com/Ryan-5853/PX4-TeleaiTrans/commit/3fef8242687cf733b590ff21c5b36779943299d1)
- 历史移植提交：[12d74dbb53](https://github.com/Li-Yuhang7683/PX4-TeleaiTrans/commit/12d74dbb53)
- [历史两个提交的源码差异（37 个文件）](https://github.com/Li-Yuhang7683/PX4-TeleaiTrans/compare/3fef8242687cf733b590ff21c5b36779943299d1...12d74dbb53)
- 目前移植分支：[`teleai/v1.16.2-endlessflight-h7ultra`](https://github.com/Ryan-5853/PX4-TeleaiTrans/tree/teleai/v1.16.2-endlessflight-h7ultra)，此前确认远程 HEAD 为 `c521c9e990b99ec07ff116ea7799a25befe81b14`；2026-11-16 串口增量尚无已提交证据。
- Ryan 后续上游：[`3dad718`](https://github.com/Ryan-5853/PX4-TeleaiTrans/commit/3dad7184942992325173f970b1b889ee7eecb414)；不属于这次已验证的串口修改。

在 **VS Code 的 WSL2 Ubuntu 终端**中检查：

```bash
cd ~/projects/PX4-H7Ultra-Ryan
source ~/venvs/px4-h7ultra/bin/activate
python --version
git branch --show-current
git rev-parse HEAD
git status --short
git diff --check
git diff --stat
git diff
```
若 `~/venvs/px4-h7ultra/bin/activate` 不存在，应先查找实际虚拟环境路径，不应误以为 Conda 环境已自动激活。

### 9.2 编译并按时间戳保存固件、源码修改和日志

在**项目根目录的 Bash 终端**运行以下整段命令；每次会新建 `build_archive/YYYYMMDD_HHMMSS/`。建议将 `build_archive/` 加入本地 `.git/info/exclude`，避免误将产物添加到 Git：

```bash
cd ~/projects/PX4-H7Ultra-Ryan
source ~/venvs/px4-h7ultra/bin/activate

# 仅本机 Git 排除，不修改需要提交的 .gitignore
if ! grep -qx 'build_archive/' .git/info/exclude 2>/dev/null; then
    printf '\nbuild_archive/\n' >> .git/info/exclude
fi

STAMP=$(date +%Y%m%d_%H%M%S)
SAVE_DIR="$PWD/build_archive/$STAMP"
mkdir -p "$SAVE_DIR"

git branch --show-current > "$SAVE_DIR/git_branch.txt"
git rev-parse HEAD > "$SAVE_DIR/git_commit.txt"
git status --short > "$SAVE_DIR/git_status.txt"
git diff --binary > "$SAVE_DIR/source_changes.patch"
python --version > "$SAVE_DIR/python_version.txt" 2>&1

# 编译输出同时显示在终端并写入 build.log
set -o pipefail
make endlessflight_h743ultra_nncontrol 2>&1 | tee "$SAVE_DIR/build.log"
BUILD_EXIT=${PIPESTATUS[0]}
echo "$BUILD_EXIT" > "$SAVE_DIR/build_exit_code.txt"

if [ "$BUILD_EXIT" -eq 0 ]; then
    FW=$(find build/endlessflight_h743ultra_nncontrol \
        -type f -name '*.px4' -print -quit)
    if [ -n "$FW" ]; then
        cp "$FW" "$SAVE_DIR/endlessflight_h743ultra_nncontrol_$STAMP.px4"
        sha256sum "$SAVE_DIR"/*.px4 > "$SAVE_DIR/firmware_sha256.txt"
        echo "编译成功，固件与日志归档：$SAVE_DIR"
    else
        echo "构建命令成功，但没有找到 .px4 文件，需要检查打包输出"
    fi
else
    echo "编译失败（退出码 $BUILD_EXIT），日志位于 $SAVE_DIR/build.log"
fi
```
**归档内容**：带时间戳的 `.px4`、`build.log`、`build_exit_code.txt`、`git_branch.txt`、`git_commit.txt`、`git_status.txt`、`source_changes.patch`、`python_version.txt`、`firmware_sha256.txt`（仅找到固件时生成）。`source_changes.patch` 只包含 Git 跟踪文件的未提交修改；若有新增未跟踪源码，须另外收录。此段是**复现操作指南**，不是本次已经生成这些归档文件的证据。

### 9.3 编译后与实机串口检查（拆桨）

```sh
# QGroundControl → MAVLink Console / NSH
gs_link status
airlink status
mavlink status
param show SER_TEL1_BAUD
param show MAV_0_CONFIG
param show MAV_1_CONFIG
param show MAV_2_CONFIG
param show GPS_1_CONFIG
param show GPS_2_CONFIG
param show NN_LQI_OUTPUT_EN
```
本次配置的**期望值**为 `gs_link: /dev/ttyS0 @ 5000000`、`airlink: /dev/ttyS2 @ 6000000`，并确认 UART3 没有其他模块占用；`GPS_1_CONFIG` / `GPS_2_CONFIG` 当前保持不启动。实际检测值只能在新固件完成编译、刷写和上电验证后补入本文。

**总结**：历史阶段已完成 Ryan PX4/LQI 框架向 H7Ultra 的 BSP 适配、QMC5883P 驱动集成、`nncontrol` 目标编译与 GS-Link 5 Mbps 旧版通信验证。**2026-11-16 新增的是本地串口配置增量**：统一 GS-Link 的 5 Mbps 参数、默认值和 QGC 选项；Airlink 在 H7Ultra 上改用 UART3 `/dev/ttyS2`、6 Mbps；GPS 保持禁用，不扩展 GNSS 编译能力。新改动已有 Git 差异与格式检查结果，但**编译、刷写、UART3 通信和飞行验证仍待实测**；LQI 核心算法及通信协议未因本次增量而更改。
