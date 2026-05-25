# Hero 工程控制链路说明

本文按当前源码整理，覆盖主控工程、任务入口、通信链路、底盘、云台、发射、自瞄、灯条和调试入口。芯片与外设初始化由 CubeMX 生成文件负责，业务代码集中在 `User` 目录。

## 工程结构

```text
Core/Src
  main.c：HAL、时钟、外设、FDCAN、UART DMA、TIM24、uproto、FreeRTOS 启动入口
  freertos.c：FreeRTOS 任务创建入口

User/APP
  gimbal_task.c：云台闭环任务，同时调度发射控制
  chassis_task.c：双舵轮底盘闭环任务
  auto_aim.c：自瞄误差缓存和开关状态
  detect_task.c：DBUS、底盘电机在线检测
  service_task.c：蜂鸣器、IMU 初始化、板载 WS2812 服务
  light_task.c：外置 CH32 灯条状态帧生成

User/APP_Support
  yaw_pitch_direct.c：云台反馈、目标、MIT 下发链路
  gimbal_behaviour.c：云台行为状态机
  shoot_task.c：摩擦轮和拨弹 MIT 控制
  chassis_behaviour.c：底盘行为状态机
  chassis_calculate.c：双舵轮逆运动学
  chassis_power_control.c：底盘电流和 PM01 功控入口
  pm01_api.c：PM01 超级电容 CAN 对象字典访问
  referee.c：裁判系统帧解析结构

User/BSP
  bsp_fdcan.c：FDCAN 收发、MIT/DJI/PM01 反馈分发
  bsp_usart.c：UART DMA 接收分发、UART 发送封装
  remote_control.c：SBUS 到 RC_ctrl_t 解析
  bsp_tim24.c：通信模块微秒时间基准

User/Devices
  hwt_imu.c：HWT101/HWT906 IMU 解析
  vofa.c：VOFA 调试输出
  ws2812.c：板载 WS2812 驱动

User/Algorithm
  pid.c / adrc.c / gravity_comp.c / user_lib.c：PID、ADRC、重力补偿、滤波和数学工具

User/Communication
  uproto + channel：USB CDC 上位机通信、云台/相机/时间同步通道

light/CH32V003F4P
  外置 10 灯 WS2812 灯板固件
```

工程主链路：

```text
遥控器 / IMU / FDCAN 反馈 / USB 通信 / 裁判系统
  -> BSP 或 Devices 解析为结构体
  -> APP 任务读取反馈和输入
  -> APP_Support 状态机生成目标量
  -> Algorithm 计算控制量
  -> BSP 打包为 CAN、UART、USB 帧
  -> 执行器反馈进入下一轮控制
```

## 启动链路

`Core/Src/main.c` 当前启动顺序：

```text
HAL_Init()
  -> SystemClock_Config()
  -> PeriphCommonClock_Config()
  -> MX_GPIO_Init()
  -> MX_DMA_Init()
  -> MX_SPI6_Init()
  -> MX_USART1_UART_Init()
  -> MX_UART7_Init()
  -> MX_FDCAN1_Init()
  -> MX_TIM6_Init()
  -> MX_FDCAN2_Init()
  -> MX_TIM12_Init()
  -> MX_FDCAN3_Init()
  -> MX_UART5_Init()
  -> MX_USART10_UART_Init()
  -> MX_SPI2_Init()
  -> MX_TIM24_Init()
  -> bsp_can_init()
  -> HAL_UARTEx_ReceiveToIdle_DMA(USART1 / UART5 / UART7 / USART10)
  -> tim24_timebase_init()
  -> proto_init_from_main()
  -> MX_FREERTOS_Init()
  -> osKernelStart()
```

`bsp_can_init()` 启动 FDCAN1、FDCAN2、FDCAN3 并打开接收中断。UART5 接收 SBUS，UART7 接收 HWT101，USART10 接收 HWT906，USART1 当前只启动 DMA 接收。

## FreeRTOS 任务

| 任务 | 优先级 | 栈 | 入口 | 周期 | 职责 |
|---|---:|---:|---|---:|---|
| `gimbalTask` | High | 1024 | `GimbalTask_Init()` | `GIMBAL_CONTROL_TIME` = 1 ms | 云台闭环、重力补偿、MIT 下发、发射调度、VOFA 输出 |
| `chassis` | High | 768 | `chassis_task()` | `osDelay(1)` | 双舵轮底盘控制、功控、电流下发 |
| `auto_aim` | Normal | 256 | `auto_aim_task()` | `AUTO_AIM_TIME` | 自瞄误差缓存、R 键软开关 |
| `COMM_APP` | 配置值 | 配置值 | `comm_app_start()` | `COMM_APP_LOOP_DELAY_MS` | USB CDC、uproto、云台/相机/时间同步通道 |
| `lightTask` | Low | 256 | `LightTask_Init()` | 50 ms | 外置灯条状态帧 |
| `detect` | Low | 128 | `detect_task()` | `DETECT_CONTROL_TIME` | DBUS 和底盘电机在线状态 |
| `service_task` | Low | 配置值 | `ServiceTask_Init()` | `SERVICE_CONTROL_TIME` | 蜂鸣器、IMU 初始化、板载灯服务 |
| `defaultTask` | Normal | 128 | `StartDefaultTask()` | 1 ms | USB_DEVICE 初始化和保留循环 |

任务创建链路：

```text
MX_FREERTOS_Init()
  -> defaultTask
  -> auto_aim_task
  -> comm_app_start()
  -> GimbalTask_Init()
  -> ServiceTask_Init()
  -> LightTask_Init()
  -> detect_task
  -> chassis_task
```

## 输入链路

遥控器链路：

```text
UART5 DMA
  -> HAL_UARTEx_RxEventCallback()
  -> sbus_to_rc(remote_buff, &rc_ctrl, &remoter)
  -> get_remote_control_point()
  -> 云台 / 底盘 / 发射 / 自瞄读取同一份 RC_ctrl_t
```

IMU 链路：

```text
UART7 DMA
  -> hwt101_rx_parse()
  -> hwt101_get_yaw_total_rad()
  -> 云台 yaw 相对底盘角计算

USART10 DMA
  -> hwt906_rx_parse()
  -> get_INS_angle_point()
  -> get_gyro_data_point()
  -> 云台绝对角、pitch/yaw 角速度、底盘姿态读取
```

FDCAN1 接收链路：

| ID | 处理函数 | 数据目标 | 用途 |
|---|---|---|---|
| `0x51` | `MITFdbData()` | `MIT_MOTOR_MEASURE[0]` | 云台 yaw MIT 反馈 |
| `0x53` | `MITFdbData()` | `MIT_MOTOR_MEASURE[2]` | 拨弹 MIT 反馈 |
| `0x205~0x208` | `get_motor_measure()` | `CHASSIS_MOTOR_MEASURE[0..3]` | 双舵轮 3508/6020 反馈 |
| `0x600~0x603` | `pm01_response_handle()` | `pm01_od` | PM01 设置回包 |
| `0x610~0x613` | `pm01_response_handle()` | `pm01_od` | PM01 状态、输入、输出、温度 |

FDCAN2 接收链路：

| ID | 处理函数 | 数据目标 | 用途 |
|---|---|---|---|
| `0x52` | `MITFdbData()` | `MIT_MOTOR_MEASURE[1]` | 云台 pitch MIT 反馈 |
| `0x201~0x203` | `get_motor_measure()` | `DJI_MOTOR_MEASURE[0..2]` | 三路摩擦轮反馈 |

FDCAN3 当前只保留通用接收缓存。

裁判系统链路：

```text
referee_data_solve(frame)
  -> 解析帧头和 cmd_id
  -> memcpy 到 game_status / robot_status / power_heat_data / shoot_data 等全局结构
  -> get_chassis_power_limit() / get_buffer_energy() / get_shooter_17mm_heat()
```

`USART1` 已启动 DMA 接收，`HAL_UARTEx_RxEventCallback()` 中 USART1 分支当前没有把数据送入 `referee_data_solve()`。

## CAN 下发链路

| 执行器 | 总线 | 下发 ID | 下发函数 | 反馈 |
|---|---|---:|---|---|
| 云台 yaw MIT | FDCAN1 | `0x01` | `CAN_cmd_MIT(&hfdcan1, DM_YAW_CAN_ID, ...)` | `0x51 -> MIT_MOTOR_MEASURE[0]` |
| 云台 pitch MIT | FDCAN2 | `0x02` | `CAN_cmd_MIT(&hfdcan2, DM_PIT_CAN_ID, ...)` | `0x52 -> MIT_MOTOR_MEASURE[1]` |
| 拨弹 MIT | FDCAN1 | `0x03` | `CAN_cmd_MIT(&hfdcan1, DM_STRUM_CAN_ID, ...)` | `0x53 -> MIT_MOTOR_MEASURE[2]` |
| 三路摩擦轮 | FDCAN2 | `0x200` | `shoot_task_send_friction_current()` | `0x201~0x203 -> DJI_MOTOR_MEASURE[0..2]` |
| 双舵轮底盘 | FDCAN1 | `0x1FF` | `CAN_cmd_CHASSIS_ALL()` | `0x205~0x208 -> CHASSIS_MOTOR_MEASURE[0..3]` |
| PM01 | FDCAN1 | `0x600~0x603` | `pm01_cmd_send()` 等 | `0x600~0x603 / 0x610~0x613 -> pm01_od` |

`CAN_cmd_CHAS_3508()` 仍保留 FDCAN1 `0x200` 调试接口，当前 `chassis_task()` 运行路径使用 `CAN_cmd_CHASSIS_ALL(0, 0, 0, 0)` 固定零电流发送。

## 云台控制链路

云台任务主循环：

```text
gimbal_task()
  -> gimbal_init()
  -> shoot_task_init()
  -> gimbal_set_mode()
  -> gimbal_feedback_update()
  -> gimbal_mode_change_control_transit()
  -> gimbal_set_control()
  -> gimbal_control_loop()
  -> gravity_comp_execute()
  -> gimbal_send_cmd()
  -> shoot_task_loop()
  -> gimbal_vofa_send_yaw_pitch_half()
```

云台行为链路：

```text
遥控器模式通道和键盘输入
  -> gimbal_behavour_set()
  -> gimbal_behaviour
  -> gimbal_behaviour_mode_set()
  -> yaw/pitch motor.mode
  -> gimbal_behaviour_control_set()
  -> add_yaw / add_pitch
```

行为与电机模式：

| 行为 | yaw 模式 | pitch 模式 | 控制来源 |
|---|---|---|---|
| `GIMBAL_ZERO_FORCE` | RAW | RAW | 输出清零 |
| `GIMBAL_INIT` | GYRO | ENCODE | 回到 `INIT_YAW_SET` / `INIT_PITCH_SET` |
| `GIMBAL_CALI` | RAW | RAW | 校准步进输出 |
| `GIMBAL_RELATIVE_ANGLE` | GYRO | ENCODE | 遥控器、鼠标、自瞄误差 |
| `GIMBAL_SPIN` | GYRO | ENCODE | 固定 yaw 增量叠加人工输入 |
| `GIMBAL_MOTIONLESS` | RAW | RAW | 输出清零 |

云台反馈定义：

```text
HWT906 yaw
  -> gimbal_yaw_motor.absolute_angle

HWT101 yaw
  -> yaw relative = HWT906 yaw - HWT101 yaw - yaw angle_offset
  -> gimbal_yaw_motor.relative_angle

MIT_MOTOR_MEASURE[1].fdb.pos
  -> pitch relative = pitch MIT pos - pitch angle_offset
  -> gimbal_pitch_motor.relative_angle

HWT906 gyro.z / pitch 轴角速度
  -> yaw gyro / pitch gyro
  -> gyro_accel
```

云台输出链路：

```text
目标角 / 反馈角 / 参考速度 / 参考加速度
  -> PID 反馈力矩
  -> 惯量前馈 + 静摩擦补偿
  -> pitch 重力补偿
  -> T_MIN/T_MAX 限幅
  -> CAN_cmd_MIT()
```

## 发射控制链路

发射控制挂在云台任务内执行：

```text
shoot_task_loop()
  -> shoot_task_set_mode()
  -> shoot_task_update_feedback()
  -> shoot_task_control_friction() 或 shoot_task_stop_friction()
  -> shoot_task_control_strum()
```

摩擦轮链路：

```text
R 键 / G 键 / 发射拨杆
  -> friction_enable
  -> SHOOT_TASK_READY_FRIC 或 SHOOT_TASK_STOP
  -> 三路摩擦轮 speed_rpm 反馈
  -> ADRC_Calc()
  -> 掉速前馈
  -> A 转 mA
  -> mA 转 DJI 电调命令
  -> FDCAN2 0x200
```

拨弹链路：

```text
鼠标左键
  -> 短按生成单步目标
  -> 长按生成持续 torque_cmd
  -> MIT_MOTOR_MEASURE[2] 反馈位置和速度
  -> 位置 PID + 前馈
  -> CAN_cmd_MIT(&hfdcan1, 0x03, torque_cmd)
```

发射保护条件：

```text
摩擦轮反馈超时 > SHOOT_FRIC_FDB_TIMEOUT
  -> shoot_task_motor_ready() 返回 false
  -> shoot_task_stop_friction()

摩擦轮温度 >= SHOOT_FRIC_TEMP_LIMIT
  -> shoot_task_motor_ready() 返回 false
  -> shoot_task_stop_friction()
```

## 底盘控制链路

当前底盘定义：

```text
CHASSIS_MODULE_NUM = 2
每个模块包含 1 个 3508 驱动轮和 1 个 6020 舵向电机
CHASSIS_MOTOR_MEASURE[0..1] -> 两个 3508 速度反馈
CHASSIS_MOTOR_MEASURE[2..3] -> 两个 6020 角度反馈
```

底盘任务主循环：

```text
chassis_task()
  -> vTaskDelay(CHASSIS_TASK_INIT_TIME)
  -> chassis_init()
  -> 等待 CHASSIS_MOTOR1_TOE~CHASSIS_MOTOR4_TOE 和 DBUS_TOE 在线
  -> chassis_set_mode()
  -> chassis_mode_change_control_transit()
  -> chassis_feedback_update()
  -> chassis_set_contorl()
  -> chassis_control_loop()
  -> 在线检查
  -> CAN_cmd_CHASSIS_ALL(0, 0, 0, 0)
  -> osDelay(1)
```

底盘行为链路：

```text
遥控器模式通道 / 键盘 X C Shift / Q
  -> chassis_behaviour_mode_set()
  -> chassis_behaviour_mode
  -> chassis_mode
  -> chassis_behaviour_control_set()
  -> vx_set / vy_set / wz_set
```

底盘行为：

| 行为 | 控制模式 | 输出定义 |
|---|---|---|
| `CHASSIS_NO_MOVE` | `CHASSIS_VECTOR_NO_MOVE` | `vx_set = 0`, `vy_set = 0`, `wz_set = 0` |
| `CHASSIS_FOLLOW_GIMBAL_YAW` | `CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW` | 遥控器/键盘给 `vx/vy`，yaw 通道给 `wz` |
| `CHASSIS_SPIN` | `CHASSIS_VECTOR_SPIN` | 遥控器/键盘给 `vx/vy`，`CHASSIS_SPIN_SPEED` 给 `wz` |
| `CHASSIS_RETURN` | `CHASSIS_VECTOR_RETURN` | 代码保留，当前触发入口处于注释状态 |

速度指令链路：

```text
chassis_rc_to_control_vector()
  -> CHASSIS_X_CHANNEL / CHASSIS_Y_CHANNEL 死区
  -> 摇杆比例映射
  -> WASD 计数斜坡
  -> first_order_filter_cali()
  -> vx_set / vy_set
  -> vector_rotate(gimbal_radian_of_ecd + 模式 offset)
```

双舵轮逆运动学：

```text
chas_inv_cal(vx_set, vy_set, wz_set)
  -> module_pos[0] = {-HALF_LENGTH,  HALF_WIDTH}
  -> module_pos[1] = { HALF_LENGTH, -HALF_WIDTH}
  -> wheel_speed[i] = sqrt(vx_total^2 + vy_total^2)
  -> wheel_angle[i] = atan2(vy_total, vx_total) - wheel_angle_offset.now[i]
  -> smooth_control()
  -> chassis_6020[i].angle_set
  -> chassis_3508[i].speed_set
```

3508 模型控制链路：

```text
speed_set - speed
  -> accel_target
  -> jerk 限制
  -> CHASSIS_MAX_ACCEL 限幅
  -> F_traction = ROBOT_MASS / 4 * model_accel[i]
  -> 摩擦补偿 + 小误差保持补偿
  -> Torque_output
  -> Current_A
  -> 20A 对应 16384 的电调命令
  -> model_3508_out[i]
```

6020 舵向链路：

```text
angle_set / angle
  -> 跨 PI 目标修正
  -> chas_6020_angle_pid[i]
  -> chas_6020_speed_pid[i]
  -> give_current
```

当前功控入口：

```text
chassis_power_control()
  -> pm01_access_poll()
  -> robot_status.chassis_power_limit 更新 PowerLimit.set_power
  -> chassis_3508[i].give_current = model_3508_out[i]
  -> chassis_6020[i].give_current = chas_6020_speed_pid[i].out
```

`Predict_Power()` 和 `Current_RestraintRelation_Calc()` 保留功率预测和电流衰减链路，当前主入口直接赋值。

## 自瞄与通信链路

自瞄误差写入链路：

```text
USB CDC / uproto / gimbal channel
  -> auto_aim_apply_delta_udeg()
  -> dyaw_udeg / dpitch_udeg 转 rad
  -> auto_aim_get_yaw_err_rad()
  -> auto_aim_get_pitch_err_rad()
  -> gimbal_set_control()
```

同一帧随后：

```text
gimbal_on_delta()
  -> gimbal_mailbox_set()
```

自瞄开关链路：

```text
R 键上升沿
  -> aim.auto_aim_flag 在 AIM_ON / AIM_OFF 间切换
  -> auto_aim_is_active()
  -> 云台目标规划叠加自瞄误差
```

通信任务链路：

```text
comm_app_task()
  -> MX_USB_DEVICE_Init()
  -> usb_cdc_port_init()
  -> ch_uproto_bind()
  -> setup_channels()
  -> usb_cdc_port_poll_rx()
  -> uproto_tick()
  -> camera_channel_tick()
  -> ch_uproto_arbiter_tick()
  -> chmgr_tick()
  -> comm_apply_host_controls()
  -> comm_publish_referee()
```

主机控制注入链路：

```text
gimbal_on_chassis()
  -> g_host_chassis
  -> comm_apply_host_controls()
  -> 写入 rc_ctrl.rc.ch[] 和 rc_ctrl.rc.s[]

gimbal_on_fire()
  -> g_host_fire
  -> comm_apply_host_controls()
  -> 写入 rc_ctrl.mouse.press_l / R/G 对应键位
```

## 在线检测与灯条

在线检测链路：

```text
detect_hook(toe)
  -> new_time = xTaskGetTickCount()
  -> error_exist = 0

detect_task()
  -> 周期扫描 error_list
  -> now - new_time > set_offline_time
  -> error_exist = 1

toe_is_error(toe)
  -> 返回 error_exist
```

当前显式刷新对象：

```text
DBUS_TOE
  -> rc_ctrl.last_fdb / remoter.sbus_recever_time / UART5 DMA 计数变化

CHASSIS_MOTOR1_TOE~CHASSIS_MOTOR4_TOE
  -> FDCAN1 0x205~0x208
```

外置灯条链路：

```text
LightTask_Init()
  -> light_task()
  -> light_render_auto()
  -> light_pack_frame()
  -> USART7_Transmit()
  -> UART7 TX PE8
  -> CH32V003F4P USART1 RX PD6
  -> WS2812_SetFromRgbBuffer()
```

灯条帧格式：

```text
AA 55 + 10 * RGB + 55 AA
总长度 34 bytes
刷新周期 LIGHT_TASK_PERIOD_MS = 50 ms
```

灯位定义：

| LED | 含义 |
|---:|---|
| 0 | DBUS 在线 |
| 1 | 底盘电机在线 |
| 2 | 摩擦轮在线和转速状态 |
| 3 | 自瞄在线和使能状态 |
| 4~6 | 底盘行为模式 |
| 7 | 云台行为模式 |
| 8 | 超级电容状态 |
| 9 | 灯条任务心跳 |

## 参数入口

| 模块 | 主要文件 | 关键宏或变量 |
|---|---|---|
| 云台 | `User/APP_Support/project_config.h` | `YAW_*_PID_*`, `PITCH_*_PID_*`, `YAW_INERTIA_KGM2`, `PITCH_GRAVITY_COMP_*`, `DM_*_ID` |
| 发射 | `User/APP_Support/shoot_task.h` | `SHOOT_FRIC_TARGET_SPEED_RPM`, `SHOOT_FRIC_MAX_CURRENT`, `SHOOT_FRIC*_FF_*`, `SHOOT_STRUM_*` |
| 底盘 | `User/APP/chassis_task.h` | `CHASSIS_VX_RC_SEN`, `CHASSIS_MAX_ACCEL`, `GM6020_MOTOR_*`, `M3508_*` |
| 车体几何 | `User/APP_Support/robot_param.h` | `HALF_LENGTH`, `HALF_WIDTH`, `CHASSIS_6020_INIT_ANGLE_*`, `CHASSIS_SPIN_SPEED` |
| 功控/电容 | `User/APP_Support/chassis_power_control.h`, `pm01_api.c` | `SET_POWER_VALUE`, `PowerLimit`, `g_cmd_set`, `g_power_set`, `ROBOT_CAP` |
| 通信 | `User/Communication/example/device/comm_app_config.h` | `COMM_APP_*`, `GIMBAL_*`, `TS_*`, `CAM_*` |
| 灯条 | `User/APP/light_task.h` | `LIGHT_LED_COUNT`, `LIGHT_UART_FRAME_BYTES`, `LIGHT_TASK_PERIOD_MS` |

## 调试入口

| 目标 | 观察变量或接口 |
|---|---|
| 云台 yaw | `relative_angle_set`, `relative_angle`, `absolute_angle_set`, `absolute_angle`, `gyro`, `static_friction_comp`, `given_current`, FDCAN1 `0x01` |
| 云台 pitch | `relative_angle_set`, `relative_angle`, `MIT_MOTOR_MEASURE[1]`, `gyro`, `static_friction_comp`, `given_current`, FDCAN2 `0x02` |
| 发射摩擦轮 | `speed_rpm`, `speed_set_rpm`, `give_current_a`, `give_current`, `ff_ticks`, FDCAN2 `0x200` |
| 拨弹 | `MIT_MOTOR_MEASURE[2].fdb.pos`, `set.POS`, `fdb.vel`, `set.TOR`, FDCAN1 `0x03` |
| 底盘 3508 | `speed_set`, `speed`, `model_accel`, `model_3508_out`, `give_current`, `CHASSIS_MOTOR_MEASURE[0..1]` |
| 底盘 6020 | `angle_set`, `angle`, `chas_6020_angle_pid[i].out`, `chas_6020_speed_pid[i].out`, `CHASSIS_MOTOR_MEASURE[2..3]` |
| 在线检测 | `error_list`, `toe_is_error(DBUS_TOE)`, `toe_is_error(CHASSIS_MOTORx_TOE)` |
| 灯条 | `light_control.leds[]`, `light_control.frame[]`, UART7 TX 波形，CH32 ACK/RUN 回包 |
| 自瞄 | `aim.online`, `aim.auto_aim_flag`, `delta_yaw_udeg`, `delta_pitch_udeg`, `auto_aim_get_*_err_rad()` |

## 构建状态

Keil 工程文件为 `MDK-ARM/CtrlBoard-H7_WS1812.uvprojx`。当前工程 IncludePath 已包含 `Core`、`Drivers`、`FreeRTOS`、`User/APP`、`User/APP_Support`、`User/BSP`、`User/Devices`、`User/Algorithm`、`User/Communication`、`USB_DEVICE` 和 USB Device Library。

最近一次可见构建日志 `MDK-ARM/CtrlBoard-H7_WS1812/CtrlBoard-H7_WS1812.build_log.htm` 显示：

```text
"CtrlBoard-H7_WS1812\CtrlBoard-H7_WS1812.axf" - 0 Error(s), 0 Warning(s).
```
