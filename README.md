# Hero

Hero 是一套面向 RoboMaster 英雄机器人控制板的嵌入式固件工程。主控使用 STM32H723VGTx，基于 STM32 HAL、FreeRTOS CMSIS-RTOS v1、FDCAN、UART DMA 和 USB CDC 实现云台、底盘、发射、自瞄通信、裁判数据、在线检测和灯板状态显示等功能；`light/CH32V003F4P` 目录包含外置 WS2812 灯板固件。

## 功能特性

- 云台控制：yaw/pitch 双轴 MIT 电机控制，支持陀螺仪绝对角、编码器相对角、RAW 输出、初始化、校准、静摩擦补偿、惯量前馈和 pitch 重力补偿。
- 发射控制：三路摩擦轮 ADRC 闭环、掉速前馈补偿、拨弹 MIT 电机单发/长按控制、反馈超时和温度保护。
- 舵轮底盘：双舵轮结构，3508 驱动轮速度控制，6020 舵向角控制，支持跟随云台、小陀螺、停止和回正相关状态。
- 自瞄通信：USB CDC + uproto + channel 框架，包含云台状态发布、自瞄增量注入、主机底盘/射击命令注入、相机触发和时间同步通道。
- 裁判与功控：裁判系统结构解析入口、PM01 超级电容对象字典访问、底盘功率限制接口。
- 在线检测：DBUS 和底盘电机在线状态检测，供底盘启动等待和灯板显示使用。
- 灯板显示：主控 UART8 输出 10 路 RGB 状态帧，外置 CH32V003F4P 固件驱动 WS2812 灯珠。
- 调试支持：VOFA 数据发送、Keil 构建日志、弹道/惯量测试数据和 MATLAB 拟合脚本。

## 技术栈

- MCU：STM32H723VGTx，Cortex-M7，LQFP100
- 主控框架：STM32CubeMX 生成工程 + STM32H7 HAL Driver
- RTOS：FreeRTOS，CMSIS-RTOS v1
- 构建工具：Keil MDK-ARM V5.27 工程文件
- 通信接口：FDCAN1/FDCAN2/FDCAN3、UART5、UART7、UART8、USART1、USART10、USB CDC HS、SPI2、SPI6、TIM24
- 控制算法：PID、ADRC、Kalman Filter、Quaternion EKF、重力补偿、低通滤波和数学工具函数
- 上位机协议：uproto、channel manager、gimbal/camera/time_sync 通道
- 外置灯板：CH32V003F4P + WS2812

## 目录结构

```text
Hero/
├── Core/                                   # STM32CubeMX 生成的主控基础工程
│   ├── Inc/                                # 外设句柄、HAL 配置、FreeRTOSConfig 和中断声明
│   │   ├── main.h                          # 全局 HAL 入口和 GPIO 宏
│   │   ├── FreeRTOSConfig.h                # FreeRTOS 裁剪配置
│   │   ├── fdcan.h / usart.h / tim.h       # FDCAN、UART、TIM 外设接口
│   │   └── stm32h7xx_it.h                  # 中断服务声明
│   └── Src/                                # 启动流程、外设初始化、任务创建和中断实现
│       ├── main.c                          # HAL、时钟、外设、CAN、UART DMA、TIM24、uproto 启动入口
│       ├── freertos.c                      # FreeRTOS 任务创建入口
│       ├── fdcan.c / usart.c / tim.c       # CubeMX 外设初始化实现
│       ├── stm32h7xx_it.c                  # 中断分发入口
│       └── system_stm32h7xx.c              # 系统时钟底层支持
├── Drivers/                                # ST 官方驱动和 CMSIS 支持包
│   ├── CMSIS/                              # Cortex-M、STM32H723 头文件和系统定义
│   └── STM32H7xx_HAL_Driver/               # HAL/LL 驱动源码和头文件
├── Middlewares/                            # 第三方和 ST 中间件
│   ├── Third_Party/FreeRTOS/Source/        # FreeRTOS 内核、CMSIS_RTOS 封装、heap_4
│   └── ST/STM32_USB_Device_Library/        # USB Device Core 和 CDC Class
├── USB_DEVICE/                             # USB CDC 设备层
│   ├── App/
│   │   ├── usb_device.c                    # USB 设备初始化入口
│   │   ├── usbd_cdc_if.c                   # CDC 收发回调和接口适配
│   │   └── usbd_desc.c                     # USB 描述符
│   └── Target/
│       └── usbd_conf.c                     # USB PCD 底层配置
├── User/
│   ├── Algorithm/                          # 控制算法和通用数学工具
│   │   ├── pid.c / pid.h                   # 位置式 PID
│   │   ├── adrc.c / adrc.h                 # 摩擦轮 ADRC 控制器
│   │   ├── gravity_comp.c / gravity_comp.h # pitch 重力补偿
│   │   ├── kalman_filter.c / *.h           # Kalman Filter
│   │   ├── QuaternionEKF.c / *.h           # 四元数 EKF 姿态解算
│   │   ├── controller.c / controller.h     # 控制器辅助接口
│   │   └── user_lib.c / user_lib.h         # 角度归一化、限幅、滤波和数学工具
│   ├── APP/                                # FreeRTOS 应用任务层
│   │   ├── gimbal_task.c / *.h             # 云台闭环主任务，调度发射控制
│   │   ├── chassis_task.c / *.h            # 双舵轮底盘主任务
│   │   ├── auto_aim.c / *.h                # 自瞄误差缓存、在线状态和软开关
│   │   ├── detect_task.c / *.h             # DBUS 和底盘电机在线检测
│   │   ├── light_task.c / *.h              # 外置灯板状态渲染和 UART8 帧发送
│   │   ├── service_task.c / *.h            # 蜂鸣器、IMU、板载灯服务入口
│   │   ├── Safewarning.c / *.h             # 安全提示/蜂鸣器相关逻辑
│   │   └── usb_task.c / *.h                # USB 任务保留入口
│   ├── APP_Support/                        # 应用支撑层和参数层
│   │   ├── project_config.h                # 全局模式、云台 PID、底盘几何、MIT ID、限位和公共参数
│   │   ├── gimbal_behaviour.c / *.h        # 云台行为状态机
│   │   ├── yaw_pitch_direct.c / *.h        # yaw/pitch 反馈、目标和 MIT 下发链路
│   │   ├── shoot_task.c / *.h              # 摩擦轮和拨弹控制
│   │   ├── chassis_behaviour.c / *.h       # 底盘行为状态机
│   │   ├── chassis_calculate.c / *.h       # 双舵轮逆运动学和角度平滑
│   │   ├── chassis_power_control.c / *.h   # 底盘电流分配和 PM01 功控入口
│   │   ├── pm01_api.c / *.h                # PM01 超级电容 CAN 对象字典访问
│   │   ├── referee.c / referee.h           # 裁判系统数据结构和解析接口
│   │   ├── target_curve.c / *.h            # 目标曲线工具
│   │   └── struct_typedef.h / protocol.h   # 基础类型和协议结构
│   ├── BSP/                                # 板级驱动和外设分发层
│   │   ├── bsp_fdcan.c / *.h               # FDCAN 收发、MIT/DJI/PM01 反馈分发、CAN 命令封装
│   │   ├── bsp_usart.c / *.h               # UART DMA ReceiveToIdle 回调、SBUS/IMU 分发、串口发送
│   │   ├── remote_control.c / *.h          # SBUS 到 RC_ctrl_t 解析
│   │   ├── bsp_tim24.c / *.h               # TIM24 微秒时间基
│   │   └── bsp_dwt.c / *.h                 # DWT 高精度计时
│   ├── Communication/                      # USB CDC 上位机通信协议栈
│   │   ├── core/                           # uproto、通道管理、CRC 和平台抽象
│   │   ├── channel/
│   │   │   ├── gimbal/                     # 云台状态发布、自瞄增量、射击/底盘命令通道
│   │   │   ├── camera/                     # 相机触发事件通道
│   │   │   └── time_sync/                  # 主机与设备时间同步通道
│   │   └── example/
│   │       ├── device/                     # STM32 设备端通信任务和 USB CDC 适配
│   │       ├── host/                       # C/C++ 主机端示例和 CMake 工程
│   │       └── shared/                     # 主机与设备共享协议 ID
│   └── Devices/                            # 具体设备驱动和调试输出
│       ├── hwt_imu.c / *.h                 # HWT101/HWT906 IMU 数据解析
│       ├── vofa.c / vofa.h                 # VOFA 调试数据发送
│       └── ws2812.c / ws2812.h             # 板载 WS2812 驱动
├── light/
│   ├── Src/                                # 简化版外置灯板固件源码
│   │   ├── main.c                          # 灯板主循环
│   │   ├── uart_proto.c                    # 主控到灯板串口帧解析
│   │   └── ws2812.c                        # WS2812 输出
│   ├── Inc/                                # 灯板公共头文件和板级配置
│   └── CH32V003F4P/                        # WCH 工程化灯板固件
│       ├── User/                           # CH32V003 用户代码、WS2812、串口协议和中断
│       ├── Peripheral/                     # CH32V00x 外设库
│       ├── Startup/                        # RISC-V 启动文件
│       ├── Ld/                             # 链接脚本
│       ├── Debug/                          # 调试串口支持
│       └── obj/                            # 构建输出
├── MDK-ARM/                                # Keil MDK-ARM 主控工程
│   ├── CtrlBoard-H7_WS1812.uvprojx         # Keil 工程文件
│   ├── CtrlBoard-H7_WS1812.uvoptx          # Keil 工程选项
│   ├── startup_stm32h723xx.s               # STM32H723 启动汇编
│   ├── CtrlBoard-H7_WS1812/                # axf、hex、map、build_log 和中间文件输出
│   ├── DebugConfig/                        # 调试器配置
│   └── RTE/                                # Keil RTE 组件配置
├── CtrlBoard-H7_WS1812.ioc                 # STM32CubeMX 工程配置
├── BUG_FIX_RECORD.md                       # 修复记录
├── control_chain.svg                       # 控制链路图
├── fit_yaw_inertia_from_vofa.m             # yaw 惯量拟合脚本
├── vofa+.csv                               # VOFA 采样数据
├── Yaw Inertia Fit.pdf                     # yaw 惯量拟合结果
├── Yaw VOFA Data Overview.pdf              # yaw 采样数据概览
├── shot.png                                # 发射/弹道相关图片
├── 调试日志.md                             # 调试记录
└── 弹道测试数据/                           # 弹道测试数据
```

## 环境要求

- Windows 开发环境。
- Keil MDK-ARM V5.27 或兼容版本。
- STM32CubeMX，版本待补充。
- STM32H7xx Device Family Pack，版本待补充。
- 调试/下载器：J-Link 或 ST-Link，具体型号待补充。
- 主控目标芯片：STM32H723VGTx。
- 外置灯板工具链：WCH CH32V003F4P 工程工具，版本待补充。
- 可选工具：Git、MATLAB、VOFA+、CMake/C++ 编译器。

## 安装步骤

1. 获取代码：

```powershell
git clone <repo-url>
cd Hero
```

2. 打开主控工程：

```text
MDK-ARM/CtrlBoard-H7_WS1812.uvprojx
```

3. 在 Keil 中安装 STM32H723VGTx 对应 Device Pack，并确认工程宏包含：

```text
USE_HAL_DRIVER,STM32H723xx,USE_PWR_LDO_SUPPLY
```

4. 使用 Keil 编译工程，输出文件位于：

```text
MDK-ARM/CtrlBoard-H7_WS1812/CtrlBoard-H7_WS1812.axf
MDK-ARM/CtrlBoard-H7_WS1812/CtrlBoard-H7_WS1812.hex
```

5. 连接调试器并下载到 STM32H723VGTx 控制板。

## 运行方式

主控上电后执行 `Core/Src/main.c`，启动链路为：

```text
HAL_Init()
-> SystemClock_Config()
-> MX_GPIO_Init() / MX_DMA_Init() / 外设初始化
-> bsp_can_init()
-> UART DMA ReceiveToIdle 启动
-> tim24_timebase_init()
-> proto_init_from_main()
-> MX_FREERTOS_Init()
-> osKernelStart()
```

FreeRTOS 创建的主要任务：

| 任务 | 入口 | 优先级 | 栈 | 周期/延时 | 职责 |
|---|---|---:|---:|---|---|
| defaultTask | `StartDefaultTask` | Normal | 128 | 1 ms | USB_DEVICE 初始化和保留循环 |
| auto_aim | `auto_aim_task` | Normal | 256 | 1 ms | 自瞄在线状态、开关和误差缓存 |
| COMM_APP | `comm_app_task` | BelowNormal | 640 | 1 ms | USB CDC、uproto、通道调度、主机命令注入 |
| gimbalTask | `gimbal_task` | High | 1024 | 1 ms | 云台闭环、重力补偿、发射调度、VOFA 输出 |
| service_task | `ServiceTask_Init` 创建 | Low | 待补充 | `SERVICE_CONTROL_TIME` | 蜂鸣器、IMU、板载灯等服务 |
| lightTask | `light_task` | Low | 256 | 50 ms | 外置灯板状态帧生成和 UART8 发送 |
| detect | `detect_task` | Low | 128 | `DETECT_CONTROL_TIME` | DBUS 和底盘电机在线检测 |
| chassis | `chassis_task` | High | 768 | 1 ms | 双舵轮底盘控制、功控、CAN 输出入口 |

## 配置说明

### 工程与外设

- `CtrlBoard-H7_WS1812.ioc`：STM32CubeMX 配置入口，包含 STM32H723VGTx、FreeRTOS、USB CDC HS、FDCAN、UART DMA、SPI 和 TIM24。
- `MDK-ARM/CtrlBoard-H7_WS1812.uvprojx`：Keil 工程入口，目标名为 `CtrlBoard-H7_WS1812`，生成 hex 文件。
- `Core/Src/main.c`：外设初始化顺序、CAN 启动、UART DMA 启动、TIM24 时间基和 uproto 初始化。
- `Core/Src/freertos.c`：任务创建入口。

### 关键外设参数

| 外设 | 配置 | 用途 |
|---|---|---|
| FDCAN1 | Classic CAN，1 Mbps | yaw MIT、拨弹 MIT、底盘电机、PM01 |
| FDCAN2 | Classic CAN，1 Mbps | pitch MIT、三路摩擦轮 |
| FDCAN3 | Classic CAN，1.25 Mbps | 预留接收入口 |
| UART5 | 100000 baud，9B，Even，2 stop，DMA RX | DBUS/SBUS 遥控器 |
| UART7 | 115200 baud，DMA RX/TX | HWT101 接收 |
| UART8 | 115200 baud，DMA TX | 外置灯板发送 |
| USART1 | 921600 baud，DMA RX/TX | 裁判系统/串口通信入口 |
| USART10 | 921600 baud，DMA RX/TX | HWT906 或扩展通信 |
| USB_DEVICE | CDC HS | 上位机通信 |
| TIM24 | 内部时钟，Prescaler 239 | 微秒时间基 |

### 接线说明

- 灯条的 RX 接开发板的 RX，第一版灯条画板丝印标错。
- HWT101 的 232 板接开发板时，RX 接 RX，TX 接 TX。

### 主要参数文件

- `User/APP_Support/project_config.h`：机器人模式、电容开关、云台 PID、底盘几何、通道映射、底盘控制参数、物理前馈参数、MIT 电机 ID、角度限位、自瞄/发射相关公共参数。
- `User/APP/chassis_task.h`：底盘控制结构体、模式枚举和任务接口。
- `User/APP_Support/shoot_task.h`：摩擦轮目标转速、电流限制、ADRC 参数、拨弹 PID 和前馈参数。
- `User/Communication/example/device/comm_app_config.h`：通信任务栈、优先级、通道 ID、USB 枚举超时、主机命令注入通道映射。
- `User/APP/light_task.h`：灯珠数量、帧长度、灯板任务周期。

### CAN ID 分配

| 设备 | 总线 | 命令 ID | 反馈 ID |
|---|---|---:|---|
| yaw MIT 电机 | FDCAN1 | `0x01` | `0x51` |
| pitch MIT 电机 | FDCAN2 | `0x02` | `0x52` |
| 拨弹 MIT 电机 | FDCAN1 | `0x03` | `0x53` |
| 摩擦轮 1/2/3 | FDCAN2 | `0x200` | `0x201` / `0x202` / `0x203` |
| 底盘电机组 | FDCAN1 | `0x1FF` | `0x205` ~ `0x208` |
| PM01 超级电容 | FDCAN1 | `0x600` ~ `0x603` | `0x600` ~ `0x603`、`0x610` ~ `0x613` |

## 常用命令

查看仓库状态：

```powershell
git status --short
```

搜索源码：

```powershell
rg "GIMBAL_CONTROL_TIME"
rg "CAN_cmd_MIT" User Core
```

用 Keil 命令行构建主控工程，`UV4.exe` 路径按本机安装位置补充：

```powershell
& "<Keil安装目录>\UV4\UV4.exe" -b "MDK-ARM\CtrlBoard-H7_WS1812.uvprojx" -j0 -o "MDK-ARM\build.log"
```

打开 STM32CubeMX 配置：

```powershell
start .\CtrlBoard-H7_WS1812.ioc
```

打开 Keil 工程：

```powershell
start .\MDK-ARM\CtrlBoard-H7_WS1812.uvprojx
```

主机端通信示例目录：

```powershell
cd .\User\Communication\example\host
```

主机端示例构建方式待补充，仓库中已存在 `build/` 输出目录。

## 开发说明

业务代码优先放在 `User` 目录，`Core`、`Drivers`、`Middlewares` 和 `USB_DEVICE` 中的 CubeMX 生成代码只在外设配置变更时同步调整。控制链路按“BSP/Devices 解析反馈 -> APP 任务读取输入 -> APP_Support 生成目标和控制量 -> Algorithm 计算 -> BSP 下发 CAN/UART/USB”的路径组织。

新增控制参数时优先放入对应模块头文件：全局、云台、底盘机械和底盘控制参数放入 `project_config.h`，发射参数放入 `shoot_task.h`，通信参数放入 `comm_app_config.h`。修改 `.ioc` 后需要用 CubeMX 重新生成代码，并检查 `USER CODE BEGIN/END` 区域内的手写逻辑是否保留。

当前 `chassis_task()` 已完成底盘目标生成、逆运动学、功控计算和电流变量写入，实际 CAN 下发入口当前发送 `CAN_cmd_CHASSIS_ALL(0, 0, 0, 0)`；恢复实车输出前需要按调试状态接入 `chassis_3508[i].give_current` 和 `chassis_6020[i].give_current`。`USART1` 已启动 DMA 接收，接收回调中的裁判系统解析接入状态待补充。
