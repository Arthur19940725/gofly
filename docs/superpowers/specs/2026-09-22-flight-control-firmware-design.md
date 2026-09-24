# gofly_fc 飞控固件设计规范

**项目：** `gofly_fc` 研究型四旋翼飞控
**日期：** 2026-09-22
**状态：** 架构已获批准，待规范审阅
**交付基线：** 自包含 STM32Cube HAL + `arm-none-eabi-gcc` 工程

## 1. 目的与边界

本规范定义与当前 KiCad 硬件相配套的第一版飞控固件。目标是交付一个可编译、可测试、可审计的 STM32F405RGT6 应用工程，覆盖硬件驱动、姿态稳定、DShot300 电机帧生成、CRSF 接收、ESC telemetry、u-blox UBX/NMEA GPS、home 点和仿真/台架优先的返航状态机。

本固件是研究原型，不是飞行认证软件，也不表示当前 PCB 可以直接接电池、装桨或执行自主返航。`GOFLY_FLIGHT_ENABLE=0`、真实电机输出和真实 RTH 授权默认关闭。没有完成电源、传感器、DShot 时序、失控保护和实机测试前，不得改变这些默认值。

### 1.1 包含范围

- STM32F405RGT6 LQFP64 的 Cube HAL 初始化和固定频率调度；
- ICM-42688-P SPI 驱动和中断采样路径；
- BMP280 I²C 驱动；
- W25Q128JV 标准 SPI 驱动；
- MAX7456 SPI/复位驱动接口，保留模拟 CVBS 语义；
- CRSF/ELRS 单线半双工接收与 CRC 校验；
- 独立 ESC telemetry 串口接收；
- UBX `NAV-PVT` 主路径与 NMEA GGA/RMC 回退；
- ADC 电池电压/电流采样和健康判断；
- 四元数姿态估计、角速度/姿态 PID、X 架混控；
- DShot300 16-bit 帧生成和受安全门控的 TIM3 输出适配；
- GPS fix、home、NED 小范围坐标、距离/方向和 RTH 意图状态机；
- 主机端纯 C 测试、确定性 HIL 场景和结构化故障日志。

### 1.2 明确不包含

- 未命名数字 VTX 的高速视频数据通道；MAX7456 只代表 5 V 模拟 CVBS OSD；
- 磁力计驱动或静止时可靠的绝对航向保持；
- 自动避障、地形跟随、航点任务、云台控制和完整地面站协议；
- 未经验证的双向 DShot；
- 真实飞行放行、飞行认证、冗余传感器或安全关键等级保证；
- 在本机不存在工具链时声称已经完成目标板编译或烧录。

## 2. 已确认的硬件契约

固件必须使用现有 KiCad 工程记录的信号命名和资源分配，不得在实现阶段静默改换 MCU 引脚。以下表格是驱动层的唯一资源基线；任何变更必须同时修改硬件映射表、Cube 配置、编译期断言和测试夹具说明。

| 功能 | STM32 资源 | 固件接口 |
|---|---|---|
| HSE | PH0/PH1 | 8 MHz 外部晶振，系统时钟须提供 USB 所需 48 MHz |
| IMU SPI | SPI1 PA5/PA6/PA7 | `IMU_SCK/MISO/MOSI` |
| IMU CS/INT | PB0 / PC4 | `IMU_CS` / `IMU_INT1` |
| Flash SPI | SPI3 PB3/PB4/PB5 | `FLASH_SCK/MISO/MOSI` |
| Flash CS | PB1 | `FLASH_CS` |
| MAX7456 SPI | SPI2 PB13/PB14/PB15 | `OSD_SCK/MISO/MOSI` |
| MAX7456 CS/reset | PB12 / PA1 | `OSD_CS` / `OSD_RESET` |
| BMP280/I²C 扩展 | I2C1 PB8/PB9 | `I2C1_SCL/SDA`，BMP280 地址 `0x76` |
| CRSF/ELRS | USART1 PA9/PA10 | `CRSF_BUS`，单线半双工，默认 420000 baud |
| GPS | USART3 PB10/PB11 | `GPS_TX/GPS_RX`，默认 57600 baud |
| 外部数字 VTX 控制 | UART4 PC10/PC11 | `VTX_MSP_TX/RX`，只作控制边界 |
| ESC telemetry | USART2 PA2/PA3 | `ESC_TELEM_RX/TX`，RX 为必需输入 |
| 电机输出 | TIM3 PC6/PC7/PC8/PC9 | `MOTOR1..4`，DShot300 候选资源 |
| 电池/电流 ADC | ADC PC0/PC1 | `VBAT_SENSE/CURRENT_SENSE`，禁止接 raw VBAT |
| USB FS | PA11/PA12 | `USB_DM/DP` |
| USB VBUS 检测 | PA8 | `USB_VBUS_SENSE`，只能通过已验证分压/比较器进入 |
| SWD | PA13/PA14/NRST | `SWDIO/SWCLK/NRST` |
| Buzzer/LED | PC5/PA0/PC13 | `BUZZER_CTRL/LED_STRIP/STATUS_LED` |

MAX7456 的 `SDOUT` 必须经过硬件确定的 5 V 到 3.3 V 保护/转换路径。由于当前资源表没有为它分配必需的 MCU 输入，固件第一版只提供 MAX7456 寄存器/复位接口和转换后测试点观测，不得自行把该信号占用到未审阅的 MCU 引脚。

TIM3 的 DMA stream/request、通道占用和中断优先级必须以 STM32F405 目标数据手册、Cube 配置和所选 HAL 版本三方核对为准。在核对完成前，DShot 硬件后端不能被真实输出授权。

## 3. 工程形态与构建

### 3.1 工程目录

```text
firmware/
├── gofly_fc.ioc
├── Makefile
├── README.md
├── Core/
│   ├── Inc/                 # Cube 生成的 HAL 配置和板级声明
│   └── Src/                 # main、时钟、GPIO、SPI、I2C、UART、ADC、TIM、DMA
├── Drivers/
│   ├── Bsp/                 # 引脚、板级时钟、故障灯和安全默认值
│   ├── Icm42688/
│   ├── Bmp280/
│   ├── W25q128/
│   ├── Max7456/
│   ├── Gps/
│   ├── Crsf/
│   ├── EscTelemetry/
│   ├── Dshot300/
│   └── AdcSense/
├── Flight/
│   ├── attitude_estimator.c
│   ├── rate_controller.c
│   ├── attitude_controller.c
│   ├── mixer_quad_x.c
│   └── flight_state.c
├── Navigation/
│   ├── gps_health.c
│   ├── geo_ned.c
│   └── rth_state.c
├── Safety/
│   ├── health_monitor.c
│   ├── output_gate.c
│   └── failsafe.c
├── App/
│   ├── scheduler.c
│   ├── state_snapshot.c
│   ├── parameters.c
│   ├── telemetry.c
│   └── app_main.c
├── tests/
│   ├── host/                # 不依赖 HAL 的纯 C 测试
│   └── hil/                 # 确定性输入场景
├── Config/
│   ├── flight_config.h
│   ├── board_map.h
│   └── safety_config.h
├── Drivers/STM32F4xx_HAL_Driver/
├── Drivers/CMSIS/
├── startup_stm32f405rgtx.s
└── STM32F405RGTX_FLASH.ld
```

HAL/CMSIS 使用 ST 官方 STM32CubeF4 版本，版本号必须写入 `README.md` 和构建输出。工程应将实际使用的 HAL/CMSIS 源码置于项目内，或者由一个明确失败的准备脚本从本机 Cube 安装复制到固定目录；不能依赖未记录的全局 include 路径。禁止把用户密钥、Cube 私有路径或机器特定绝对路径写入工程。

由于当前环境尚未发现 `arm-none-eabi-gcc`、`make`、CMake 或 STM32CubeMX，实施阶段必须先执行工具链探测。探测失败时仍可完成源码和主机测试，但必须把目标板构建标记为未验证，不得伪造成功日志。

### 3.2 构建配置

- `bench`：默认配置，`GOFLY_FLIGHT_ENABLE=0`、`GOFLY_REAL_OUTPUT=0`、DShot 后端可生成帧但输出至测试/观测适配器；
- `target`：启用真实 HAL 外设句柄和已核对的 DMA 配置，但仍保持真实输出关闭，供示波器/逻辑分析仪验证；
- `flight`：仅在用户明确修改配置、完成硬件检查并满足运行时安全门控后可构建，默认不提供；
- `host-test`：使用主机编译器编译纯算法/协议模块，不引入 HAL；
- 所有配置在编译时打印 MCU、HAL 版本、板级映射版本、安全开关和 DShot 后端状态。

## 4. 软件分层与接口

固件采用无 RTOS、固定内存、无动态分配的确定性架构。中断只搬运数据、置位事件或更新时间戳，不执行浮点导航、PID 或复杂协议处理。

```text
HAL ISR/DMA
  -> 有界环形缓冲区 / 采样就绪事件
  -> 驱动解析与设备状态
  -> 原子化状态快照
  -> 姿态估计与控制器
  -> 安全门控
  -> DShot300 / 外部遥测
```

### 4.1 Core

Cube 生成时钟、GPIO、SPI、I²C、UART、ADC、TIM、DMA、USB 和中断初始化。应用只通过板级句柄和薄适配函数访问 HAL，不能在 Flight/Navigation 层直接调用 `HAL_*`。所有 HAL 回调必须把错误转成统一事件并记录设备、时间戳和计数器。

### 4.2 Drivers

每个驱动都分为无 HAL 的协议/寄存器逻辑和薄硬件传输适配层。传输 API 使用超时、有界长度和显式结果码；禁止在中断回调中阻塞等待。

- **ICM-42688-P**：SPI 四线模式，启动 WHO_AM_I、配置量程/ODR/滤波/INT1，1 kHz 采样；VDD/VDDIO 均为 `IMU_3V3`；
- **BMP280**：I²C `0x76`，验证芯片 ID、校准系数和补偿结果；不支持运行时断电后仍驱动总线；
- **W25Q128JV**：标准 SPI，等待上电写禁止时间，轮询 BUSY，`/WP` 与 `/HOLD` 保持安全状态；
- **MAX7456**：5 V 模拟 CVBS OSD，SPI 不超过约定上限，复位脉宽和晶振状态由驱动检查；不实现数字视频；
- **CRSF**：有界帧解析、长度/CRC/帧类型检查、通道解码、链路年龄和 failsafe 标记；
- **GPS**：UBX `NAV-PVT` 主解析，NMEA GGA/RMC 回退，协议层输出统一 GPS fix；
- **DShot300**：独立帧编码器、CRC 和油门范围检查；TIM3/DMA 后端只接收已授权的四路帧；
- **ESC telemetry**：独立串口输入、帧校验、数据年龄与异常范围检查；
- **ADC**：采样平均/滤波、比例换算和范围检查，所有比例常数为配置值并带单位。

### 4.3 App 与调度

调度器使用单调递增的 HAL tick 或硬件定时事件，以固定周期调用任务。任务不得通过无限循环阻塞其他任务；每个任务有执行预算和过期计数器。共享状态采用双缓冲或临界区保护的快照，不在控制环中读取半更新结构。

建议周期：

| 任务 | 周期 | 说明 |
|---|---:|---|
| IMU 读取/角速度更新 | 1 kHz | 数据新鲜度是输出授权条件 |
| 姿态估计 | 500 Hz | 四元数与置信度 |
| 角速度 PID/电机帧 | 1 kHz | 过期即清零输出 |
| CRSF 消费 | 200 Hz | 接收由 DMA/中断填充 |
| GPS 解析/导航 | 100 Hz | 无阻塞解析 |
| ADC/ESC/健康监测 | 50–100 Hz | 记录故障和阈值状态 |
| 参数/日志/USB 状态 | 10 Hz | 事件触发可立即发送 |

## 5. 姿态与控制设计

### 5.1 坐标和状态

- 世界坐标为 NED；机体坐标为 FRD；
- 姿态内部使用归一化四元数；对异常范数、NaN、无穷值执行拒绝并转为健康故障；
- IMU 原始单位在驱动边界转换为 SI 单位，控制层不处理寄存器量程换算；
- 加速度只作为重力方向观测，不把强线性加速度误认为姿态误差。

### 5.2 估计器

采用 Mahony 风格的陀螺积分加加速度校正。校正权重由加速度模长偏离重力的程度限制；IMU 数据过期、饱和或校验失败时，姿态置信度下降并清除输出授权。没有磁力计时，横滚/俯仰可长期校正，偏航只能依靠陀螺积分；GPS 航迹角只在水平速度超过阈值且 GPS 健康时作为可选辅助，不在静止时伪造磁航向。

### 5.3 控制器与混控

控制链为：

```text
CRSF/导航目标
  -> 姿态外环（可选 Angle 模式）
  -> 角速度目标
  -> 1 kHz 角速度 PID
  -> Quad-X mixer
  -> 油门/输出限制
  -> Safety output gate
  -> DShot300
```

第一版支持 Rate 和 Angle 两种模式。PID 必须有积分限幅、输出限幅、饱和时积分处理、D 项低通/油门相关削弱和故障清零。混控使用显式电机顺序、旋转方向和符号配置，不把连接器顺序硬编码为物理编号；启动自检和遥测必须能输出当前映射。

## 6. GPS、home 与 RTH

### 6.1 GPS 统一数据模型

协议解析器输出统一结构：UTC 时间、纬度、经度、MSL 高度、速度、航迹角、fix 类型、卫星数、水平精度、数据时间戳和校验状态。GPS 健康由 fix、精度、年龄、跳变和卫星数分别判定；旧数据不会无限期沿用。

### 6.2 Home 策略

状态从 `NO_FIX` 进入 `HOME_CANDIDATE`，在连续样本满足稳定性、精度和卫星阈值后才允许锁定。home 包含经纬度、参考高度、锁定时间和质量指标；锁定后默认不自动移动。GPS 跳变、fix 降级或年龄超时会使导航退回安全状态，而不是继续使用未经确认的旧坐标。

WGS-84 经纬度在 home 附近转换为 NED 米制坐标，用于距离和方向。实现必须避免把经纬度差直接当成米数，并对极小距离、无效纬度/经度和数值溢出做边界处理。

### 6.3 RTH 状态机

```text
RTH_DISABLED
  -> GPS_UNAVAILABLE
  -> HOME_READY
  -> RTH_REQUESTED
  -> RTH_CLIMB_INTENT
  -> RTH_RETURN_INTENT
  -> RTH_ARRIVAL_INTENT
  -> RTH_ABORTED / RTH_COMPLETE
```

触发源包括显式 RTH 请求和 RC 链路失联；每次转换必须记录原因。第一版实现爬升、返航和到达的导航意图（目标高度、目标速度、目标方向、剩余距离）以及超时/取消/失败原因，但默认不把意图直接授权给真实电机输出。

RTH 只有在 GPS、home、姿态、控制周期、电池和 RC/failsafe 条件均满足时才能进入意图状态。没有磁力计时，若偏航置信度低于阈值，RTH 应进入 `RTH_ABORTED` 或仅保留观测输出，不能声称已完成自主返航。仿真输入可推进所有状态并生成确定性日志。

## 7. 安全与故障处理

安全决策使用显式优先级：

```text
硬件/时钟故障
  > IMU 过期或无效
  > 控制周期超时
  > RC 链路失效
  > GPS 失效
  > 电池异常
  > 普通姿态/导航请求
```

输出授权是所有条件的交集：编译期授权、运行时 bench/flight 模式、上电自检、IMU 新鲜、姿态有效、RC 有效、明确 ARM、无高优先级故障、DShot 后端健康。任一条件失效都必须让四路输出进入安全值并清除 PID 积分。恢复必须重新经过待解锁流程，不能在故障消失后自动重启电机。

故障记录包含代码、来源、单调时间戳、第一次发生时间、最近发生时间、计数器和恢复状态。环形日志为固定大小，溢出时丢弃最旧记录并设置溢出标志；不能动态申请内存或在错误路径阻塞。

## 8. 主机测试与 HIL

所有协议、地理、估计器、PID、混控、状态机和安全门控逻辑必须可在主机编译，不依赖 HAL、寄存器或中断。测试至少覆盖：

- DShot300 16 位帧、油门边界和 CRC；
- CRSF 合法帧、CRC 错误、截断帧、噪声流、链路超时和 failsafe；
- UBX/NMEA 校验、分帧、混合协议流、坏长度和时间年龄；
- 四元数归一化、静止校正、异常输入和加速度扰动；
- PID 积分限幅、饱和恢复和周期异常；
- Quad-X 符号、顺序和油门限制；
- WGS-84 小范围 NED 距离/方向及边界值；
- home 锁定、GPS 降级、RTH 请求/取消/超时/中止；
- 任一安全故障都屏蔽非零电机输出；
- 固定时间步长 HIL：RC 丢失、GPS 丢失、低电压、IMU 超时、DShot 后端错误。

目标板验证顺序为：无桨电源与电源轨、SWD、时钟、USB、IMU WHO_AM_I、BMP280 ID、Flash 状态、串口回环、DShot 逻辑分析、无桨 ESC、最后才是受控试验。代码和文档不得跳过这些门槛。

## 9. 验收标准

### 9.1 工程

- 工程包含 `.ioc`、启动文件、链接脚本、HAL/CMSIS 版本记录和可复现构建入口；
- 默认 bench 配置可在有工具链的环境中编译；
- 主机测试无需 STM32 工具链即可运行；
- 构建输出明确显示安全开关和未验证的 DMA/硬件后端状态；
- 无动态内存、未界定的环形缓冲区或无界协议解析。

### 9.2 功能

- 设备驱动有 ID/状态/超时和明确错误码；
- IMU、姿态、PID、混控、DShot 帧、CRSF、GPS、home、RTH 和 failsafe 均有主机测试；
- 四路电机输出在任何默认配置或健康条件不足时保持安全；
- RTH 在无 home、GPS 不健康、姿态/偏航置信度不足时不进入真实输出路径；
- 现有硬件资源表与固件板级映射一致，不出现 PE 管脚或重复外设分配。

### 9.3 明确未验证事项

- 当前工作站没有检测到 `arm-none-eabi-gcc`、Make、CMake 或 CubeMX；
- STM32F405 TIM3/DMA 的最终 stream/request 组合尚未由目标 HAL/Cube 配置核对；
- TPS54360 输入保护、电源瞬态和热设计尚未通过台架测试；
- CRSF/ELRS、GPS、ESC telemetry、VTX 的具体硬件电平尚未绑定到型号；
- PCB 当前仍有布局/庭院冲突，未完成布线/DRC；
- 当前没有磁力计，偏航和真实自主返航能力受限；
- 不得把该固件或 PCB 描述为 flight-ready。

## 10. 变更控制

硬件引脚、外设、协议默认值、安全门控和 RTH 行为属于接口契约。改变它们必须同时更新 `gofly_fc.ioc`、`Config/board_map.h`、测试夹具、主机测试和本规范。任何为了让测试通过而降低安全门槛、绕过 CRC/超时或默认开启真实电机输出的改动都不接受。
