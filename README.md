# Hero 工程控制链路说明

本文只说明业务代码和控制链路。芯片启动、CubeMX 外设初始化、HAL 驱动细节只保留与业务链路直接相关的入口。

## 1. 工程分层

```text
Core/Src
  -> main.c 完成外设初始化、CAN 启动、UART DMA 接收启动、FreeRTOS 任务创建入口

User/BSP
  -> 板级通信封装：FDCAN 收发、UART DMA 分发、遥控器解析、DWT/定时器辅助

User/Devices
  -> 设备数据解析：HWT101/HWT906 IMU、VOFA 调试、WS2812 灯带

User/APP
  -> 业务任务和状态机：云台、底盘、发射、自瞄、检测、裁判、PM01、服务任务

User/Algorithm
  -> 控制算法和数学工具：PID、ADRC、重力补偿、滤波、CMSIS-DSP、user_lib
```

工程的主控制路径是：

```text
遥控器 / IMU / CAN 电机反馈 / 裁判系统 / 自瞄通信
  -> BSP 或 Devices 解析成结构化数据
  -> APP 状态机选择模式并生成目标值
  -> Algorithm 计算控制输出
  -> BSP 将控制量打包为 CAN/UART/USB 数据帧
  -> 执行器反馈再次进入下一轮控制
```

## 2. 底盘移植状态

本次底盘移植后的核心文件位置：

```text
User/APP/chassis_behaviour.c / .h
User/APP/chassis_calculate.c / .h
User/APP/chassis_power_control.c / .h
User/APP/chassis_task.c / .h
```

底盘依赖文件位置：

```text
User/APP/CAN_receive.h
User/APP/INS_task.h
User/APP/detect_task.c / .h
User/APP/pm01_api.c / .h
User/APP/protocol.h
User/APP/referee.c / .h
User/APP/robot_param.h
User/APP/struct_typedef.h
User/APP/voltage_task.c / .h
User/Algorithm/user_lib.c / .h
User/Algorithm/Include/arm_math.h
User/Algorithm/Include/arm_const_structs.h
User/Algorithm/Include/arm_common_tables.h
```

复用关系：

```text
FDCAN 收发
  -> 复用 User/BSP/bsp_fdcan.c / .h
  -> 增加底盘 3508、底盘 6020、PM01 分发与下发函数

UART 与遥控器
  -> 复用 User/BSP/bsp_usart.c 和 User/BSP/remote_control.c
  -> 底盘、云台、发射读取同一个 rc_ctrl

IMU
  -> 复用 User/Devices/hwt_imu.c / .h
  -> INS_task.h 提供兼容 include，实际姿态指针来自 HWT906 导出接口

算法
  -> 复用 User/Algorithm/pid.c / .h
  -> 复用 User/Algorithm/user_lib.c / .h
  -> 复用 CMSIS-DSP 的 arm_math 接口
```

Keil 工程配置：

```text
MDK-ARM/CtrlBoard-H7_WS1812.uvprojx
  -> 已加入 chassis_*.c、detect_task.c、pm01_api.c、referee.c、voltage_task.c、user_lib.c
  -> IncludePath 已加入 ../User/Algorithm/Include
  -> Define 已加入 ARM_MATH_CM7
```

`ARM_MATH_CM7` 定义当前芯片使用 Cortex-M7 内核。`arm_math.h` 需要这个宏选择 `core_cm7.h`，底盘运动学中的 `arm_sin_f32()`、`arm_cos_f32()`、`arm_sqrt_f32()` 依赖这条配置。

## 3. 启动链路

### 3.1 硬件与通信入口

文件：`Core/Src/main.c`

```text
HAL_Init()
  -> SystemClock_Config()
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
  -> HAL_UARTEx_ReceiveToIdle_DMA(...)
  -> MX_FREERTOS_Init()
  -> osKernelStart()
```

`bsp_can_init()` 启动 FDCAN1、FDCAN2、FDCAN3，并打开接收中断。UART DMA 接收启动后，UART 空闲中断会把不同串口的数据分发到遥控器和 IMU 解析器。

### 3.2 FreeRTOS 任务入口

文件：`Core/Src/freertos.c`

```text
MX_FREERTOS_Init()
  -> defaultTask：空闲延时任务
  -> auto_aim_task：自瞄数据状态更新
  -> comm_app_start()：USB/通信协议任务
  -> GimbalTask_Init()：创建云台任务
  -> ServiceTask_Init()：创建服务任务
  -> detect_task：在线检测任务
  -> chassis_task：底盘任务
```

任务职责：

| 任务 | 优先级 | 主要文件 | 控制周期 | 职责 |
|---|---:|---|---:|---|
| `gimbal_task` | High | `User/APP/gimbal_task.c` | `GIMBAL_CONTROL_TIME` | 云台闭环、发射任务、VOFA 输出 |
| `chassis_task` | High | `User/APP/chassis_task.c` | 当前 `osDelay(1)` | 舵轮底盘闭环、功率入口、CAN 下发 |
| `auto_aim_task` | Normal | `User/APP/auto_aim.c` | `AUTO_AIM_TIME` | 自瞄误差滤波和角度增量生成 |
| `detect_task` | Low | `User/APP/detect_task.c` | `DETECT_CONTROL_TIME` | 遥控器和底盘电机在线状态判断 |
| `service_task` | Low | `User/APP/service_task.c` | `SERVICE_CONTROL_TIME` | 蜂鸣器、IMU 解析结构初始化、灯带刷新 |
| `defaultTask` | Normal | `Core/Src/freertos.c` | 1 ms | 保留任务 |

## 4. 输入数据链路

### 4.1 遥控器输入链路

文件：`User/BSP/bsp_usart.c`、`User/BSP/remote_control.c`

```text
UART5 DMA 接收 SBUS 数据
  -> HAL_UARTEx_RxEventCallback()
  -> sbus_to_rc(remote_buff, &rc_ctrl, &remoter)
  -> rc_ctrl.rc.ch[] / rc_ctrl.rc.s[] / rc_ctrl.mouse / rc_ctrl.key
  -> get_remote_control_point()
  -> 云台、底盘、发射、自瞄读取同一个遥控器结构体
```

遥控器数据是人工输入的统一入口。云台通过 `YAW_CHANNEL`、`PITCH_CHANNEL`、鼠标和键盘计算 yaw/pitch 目标增量；底盘通过 `CHASSIS_X_CHANNEL`、`CHASSIS_Y_CHANNEL`、`CHASSIS_WZ_CHANNEL` 和 WASD 键计算 `vx_set`、`vy_set`、`wz_set`；发射通过 `R`、`G` 和发射拨杆决定摩擦轮启停；自瞄通过 `R` 键切换自瞄软件使能。

检测链路：

```text
rc_ctrl.last_fdb
  -> detect_task 中刷新 DBUS_TOE 的 new_time
  -> toe_is_error(DBUS_TOE)
  -> 底盘任务决定发送真实电流或零电流
```

### 4.2 IMU 输入链路

文件：`User/BSP/bsp_usart.c`、`User/Devices/hwt_imu.c`

```text
UART7 DMA 接收 HWT101
  -> hwt101_rx_parse()
  -> hwt101_info
  -> hwt101_get_yaw_total_rad()
  -> 云台 yaw 相对底盘角计算

USART10 DMA 接收 HWT906
  -> hwt906_rx_parse()
  -> hwt906_info
  -> get_INS_angle_point()
  -> get_gyro_data_point()
  -> get_accel_data_point()
  -> 云台和底盘读取姿态、角速度、加速度
```

HWT101 定义为底盘 yaw 参考源。HWT906 定义为云台姿态源，导出连续 yaw、pitch、roll，角速度 wx/wy/wz，以及线加速度 ax/ay/az。`yaw_pitch_direct.c` 里的 `gimbal_feedback_update()` 用 HWT906 yaw 与 HWT101 yaw 计算云台相对底盘 yaw，用 HWT906 gyro.z/gyro.y 作为 yaw/pitch 角速度反馈。

### 4.3 FDCAN 输入链路

文件：`User/BSP/bsp_fdcan.c`

FDCAN1 接收：

```text
0x201 ~ 0x204
  -> get_motor_measure(&CHASSIS_MOTOR_MEASURE[0..3])
  -> detect_hook(CHASSIS_MOTOR1_TOE..CHASSIS_MOTOR4_TOE)
  -> chassis_task 读取 4 个 3508 反馈

0x600 / 0x601 / 0x602 / 0x603 / 0x610 / 0x611 / 0x612 / 0x613
  -> pm01_response_handle()
  -> pm01_od 更新 PM01 状态、输入功率、电压、电流、输出功率、电压、电流、温度
```

FDCAN2 接收：

```text
DM_YAW_MASTER_ID(0x51) / DM_PIT_MASTER_ID(0x52)
  -> MITFdbData()
  -> MIT_MOTOR_MEASURE[0..1]
  -> yaw/pitch 云台 MIT 反馈

CAN_FRIC1_ID(0x201) / CAN_FRIC2_ID(0x202) / CAN_FRIC3_ID(0x203) / CAN_STRUM_ID(0x204)
  -> get_motor_measure(&DJI_MOTOR_MEASURE[0..3])
  -> shoot_task 读取摩擦轮反馈

0x205 ~ 0x208
  -> get_motor_measure(&CHASSIS_MOTOR_MEASURE[4..7])
  -> detect_hook(CHASSIS_MOTOR5_TOE..CHASSIS_MOTOR8_TOE)
  -> chassis_task 读取 4 个 6020 舵向反馈
```

FDCAN3 当前只提供通用接收缓存，没有接入核心控制链路。

### 4.4 裁判系统链路

文件：`User/APP/referee.c`、`User/APP/protocol.h`、`User/BSP/bsp_usart.c`

```text
referee_data_solve(frame)
  -> 解析 5 字节帧头
  -> 读取 cmd_id
  -> memcpy 到 game_status / robot_status / power_heat_data / shoot_data 等全局结构
  -> get_chassis_power_limit() / get_buffer_energy() / get_shooter_17mm_heat()
```

当前代码中 `referee_data_solve()` 已完整保留解析入口和数据结构，`USART1` 的接收回调没有把 DMA 数据分发给裁判解析器。功率链路会读取 `robot_status.chassis_power_limit`，因此裁判系统接入后会影响 `PowerLimit.set_power`。

### 4.5 PM01 超级电容链路

文件：`User/APP/pm01_api.c`、`User/BSP/bsp_fdcan.c`

```text
chassis_power_control()
  -> pm01_access_poll()
  -> pm01_cmd_send(0x600)
  -> pm01_power_set(0x601)
  -> pm01_voltage_set(0x602)
  -> pm01_current_set(0x603)
  -> FDCAN1 发送配置帧
  -> FDCAN1 接收 0x600/0x601/0x602/0x603/0x610/0x611/0x612/0x613
  -> pm01_response_handle()
  -> pm01_od 保存 PM01 反馈
```

`pm01_access_poll()` 受 `ROBOT_CAP == Cap_on` 条件控制。打开后每 100 次调用轮询写入 PM01 的命令、功率、电压、电流设定值。底盘行为层用 `Q` 键在 `SUPER_CAP_PREPARED` 与 `SUPER_CAP_USING` 之间切换 `super_cap_mode`。

## 5. 云台控制链路

### 5.1 云台任务主循环

文件：`User/APP/gimbal_task.c`、`User/APP/yaw_pitch_direct.c`、`User/APP/gimbal_behaviour.c`

```text
gimbal_task()
  -> gimbal_init()
  -> shoot_task_init()
  -> 循环：
       gimbal_set_mode()
    -> gimbal_feedback_update()
    -> gimbal_mode_change_control_transit()
    -> gimbal_set_control()
    -> gimbal_control_loop()
    -> gravity_comp_execute()
    -> gimbal_send_cmd()
    -> shoot_task_loop()
    -> VOFA_Send6()
```

云台任务是云台闭环和发射闭环的共同调度入口。云台每个周期先确定行为，再刷新反馈，再生成目标值，再计算 torque 输出，再把 MIT 命令发给 yaw/pitch 电机。

### 5.2 云台初始化链路

```text
gimbal_init()
  -> memset(gimbal_control)
  -> get_remote_control_point()
  -> get_INS_angle_point()
  -> get_gyro_data_point()
  -> get_accel_data_point()
  -> 初始化 yaw/pitch PID
  -> 写入 yaw/pitch 相对角限位与惯量
  -> Motor_MIT_MODE(&hfdcan2, DM_YAW_CAN_ID / DM_PIT_CAN_ID)
  -> Motor_ENABLE(&hfdcan2, DM_YAW_CAN_ID / DM_PIT_CAN_ID)
  -> gimbal_feedback_update()
  -> 当前角度写入 set 值
  -> gravity_comp_init()
```

初始化的结果是：遥控器、IMU、MIT 反馈指针被绑定，yaw/pitch 电机进入 MIT 模式并使能，PID 与重力补偿参数进入可运行状态。

### 5.3 云台行为状态机

文件：`User/APP/gimbal_behaviour.c`

```text
遥控器拨杆 / 键盘输入
  -> gimbal_behavour_set()
  -> gimbal_behaviour
  -> gimbal_behaviour_mode_set()
  -> yaw/pitch motor.mode
```

行为含义：

| 行为 | 电机模式 | 输出来源 |
|---|---|---|
| `GIMBAL_ZERO_FORCE` | RAW | yaw/pitch 输出 0 |
| `GIMBAL_INIT` | yaw GYRO，pitch ENCODE | 回到初始化角 |
| `GIMBAL_CALI` | RAW | 校准步骤直接给定输出 |
| `GIMBAL_ABSOLUTE_ANGLE` | yaw GYRO，pitch ENCODE | 遥控器/鼠标角度增量 |
| `GIMBAL_RELATIVE_ANGLE` | yaw GYRO，pitch ENCODE | 遥控器/鼠标角度增量 |
| `GIMBAL_MOTIONLESS` | RAW | yaw/pitch 输出 0 |
| `GIMBAL_SPIN` | yaw GYRO，pitch ENCODE | 固定 yaw 增量叠加人工输入 |

联锁接口：

```text
gimbal_cmd_to_shoot_stop()
  -> GIMBAL_INIT / GIMBAL_CALI / GIMBAL_ZERO_FORCE / GIMBAL_MOTIONLESS
  -> shoot_task_set_mode() 强制 STOP

gimbal_cmd_to_chassis_stop()
  -> 提供底盘停机需求接口
```

### 5.4 云台反馈链路

文件：`User/APP/yaw_pitch_direct.c`

```text
HWT906 angle yaw
  -> gimbal_yaw_motor.absolute_angle

HWT101 chassis yaw
  -> yaw relative = HWT906 yaw - HWT101 yaw - yaw angle_offset
  -> gimbal_yaw_motor.relative_angle

MIT_MOTOR_MEASURE[GIMBAL_PITCH_MIT_INDEX].fdb.pos
  -> pitch relative = pitch MIT pos - pitch angle_offset
  -> gimbal_pitch_motor.relative_angle

HWT906 gyro.z / gyro.y
  -> yaw gyro / pitch gyro
  -> gyro_accel 差分
```

yaw 的绝对角来自 HWT906，yaw 的相对角来自 HWT906 与 HWT101 的差值。pitch 的角度来自 MIT 电机位置反馈。这个定义让云台 yaw 能跟随世界坐标姿态，也能计算相对底盘角。

### 5.5 云台目标生成链路

```text
gimbal_behaviour_control_set()
  -> gimbal_read_manual_input()
  -> add_yaw / add_pitch
  -> gimbal_set_control()
  -> yaw absolute_angle_set / relative_angle_set
  -> pitch absolute_angle_set / relative_angle_set
  -> gimbal_feedforward_track_target()
```

手动输入定义：

```text
yaw 增量 = 遥控器 yaw 通道 * YAW_RC_SEN - mouse.x * YAW_MOUSE_SEN
pitch 增量 = 遥控器 pitch 通道 * PITCH_RC_SEN + mouse.y * PITCH_MOUSE_SEN
```

自瞄开启时，`gimbal_yaw_absolute_angle_limit()` 和 `gimbal_pitch_relative_angle_limit()` 读取 `auto_aim_get_yaw_err_rad()`、`auto_aim_get_pitch_err_rad()` 中的最新误差，再用限速、限加速度的轨迹规划更新目标角。

### 5.6 云台闭环输出链路

```text
目标角 / 反馈角
  -> gimbal_motor_absolute_angle_control() 或 gimbal_motor_relative_angle_control()
  -> gimbal_pid_calc() 或 gimbal_calc_angle_speed_torque()
  -> gimbal_calc_feedforward()
  -> current_set = pid_torque + ff_torque
  -> given_current = clamp(current_set, T_MIN, T_MAX)
  -> gravity_comp_execute()
  -> gimbal_send_cmd()
  -> CAN_cmd_MIT(&hfdcan2, DM_YAW_CAN_ID, ..., yaw torque)
  -> CAN_cmd_MIT(&hfdcan2, DM_PIT_CAN_ID, ..., pitch torque)
```

`gravity_comp_execute()` 只作用于 pitch 轴，基于 pitch 角、等效质量、质心前向距离、质心上向距离计算重力补偿 torque，并受 `PITCH_GRAVITY_COMP_OUTPUT_LIMIT` 限制。

## 6. 发射控制链路

### 6.1 发射调度入口

文件：`User/APP/shoot_task.c`

```text
gimbal_task()
  -> shoot_task_loop()
  -> shoot_task_set_mode()
  -> shoot_task_update_feedback()
  -> shoot_task_control_friction() 或 shoot_task_stop_friction()
  -> shoot_task_send_friction_current()
```

发射模块没有创建独立 FreeRTOS 任务。它挂在云台任务周期里执行，因此发射闭环与云台闭环同步。

### 6.2 发射模式链路

```text
R 键上升沿
  -> friction_enable = true

G 键上升沿
  -> friction_enable = false

发射拨杆下档
  -> friction_enable = true

发射拨杆中档
  -> friction_enable = false

gimbal_cmd_to_shoot_stop() == true
  -> mode = SHOOT_TASK_STOP

friction_enable == true
  -> mode = SHOOT_TASK_READY_FRIC
```

`SHOOT_TASK_STOP` 清零目标转速、电流输出、前馈状态，并发送三路零电流。`SHOOT_TASK_READY_FRIC` 执行三路摩擦轮速度闭环。

### 6.3 发射反馈链路

```text
FDCAN2 接收 CAN_FRIC1_ID / CAN_FRIC2_ID / CAN_FRIC3_ID
  -> DJI_MOTOR_MEASURE[0..2]
  -> shoot_task_update_feedback()
  -> speed_rpm = measure->speed_rpm * direction
  -> speed_mps = speed_rpm * SHOOT_FRIC_RPM_TO_MPS
  -> shoot_task_motor_update_current_physics()
```

`shoot_task_motor_ready()` 使用 `last_fdb_time` 与温度判断摩擦轮反馈有效性。任意一路反馈超时或过温，`shoot_task_control_friction()` 会进入停机输出。

### 6.4 发射速度闭环链路

```text
SHOOT_FRIC_TARGET_SPEED_RPM
  -> shoot_task_motor_calc()
  -> ADRC_Calc(speed_rpm, target_speed_rpm)
  -> current_output_a * direction
  -> 掉速前馈判断
  -> shoot_task_motor_finalize_current()
  -> A 转 mA
  -> mA 转 DJI 电调命令
  -> FDCAN2 发送 0x200
```

ADRC = Active Disturbance Rejection Control，当前用于摩擦轮速度环。它把速度误差和扰动估计转换为电流输出。掉速前馈由 `SHOOT_FRIC*_FF_*` 参数控制，用于摩擦轮速度突然下降时短时叠加固定电流。

## 7. 底盘控制链路

### 7.1 底盘任务主循环

文件：`User/APP/chassis_task.c`

```text
chassis_task()
  -> vTaskDelay(CHASSIS_TASK_INIT_TIME)
  -> chassis_init()
  -> 等待 8 个底盘电机与 DBUS 在线
  -> 循环：
       chassis_set_mode()
    -> chassis_mode_change_control_transit()
    -> chassis_feedback_update()
    -> chassis_set_contorl()
    -> chassis_control_loop()
    -> 在线检查
    -> CAN_cmd_CHAS_6020()
    -> CAN_cmd_CHAS_3508()
    -> osDelay(1)
```

底盘控制对象是四舵轮：4 个 3508 驱动轮电机负责轮速，4 个 6020 舵向电机负责轮向。反馈数组 `CHASSIS_MOTOR_MEASURE[0..3]` 对应 3508，`CHASSIS_MOTOR_MEASURE[4..7]` 对应 6020。

### 7.2 底盘初始化链路

```text
chassis_init()
  -> chassis_RC = get_remote_control_point()
  -> chassis_INS_angle = get_INS_angle_point()
  -> chassis_yaw_motor = get_yaw_motor_point()
  -> chassis_pitch_motor = get_pitch_motor_point()
  -> chassis_3508[i].chassis_motor_measure = get_chassis_motor_measure_point(i)
  -> chassis_6020[i].chassis_motor_measure = get_chassis_motor_measure_point(i + 4)
  -> model_3508_out[i] = 0
  -> model_accel[i] = 0
  -> 初始化 6020 angle PID / speed PID
  -> 初始化回正 PID 与跟随 PID
  -> 初始化 vx/vy 一阶滤波器
  -> 设置最大速度
  -> chassis_wheel_angle_offset_init()
  -> chassis_feedback_update()
```

3508 当前使用物理模型电流控制链路，不再初始化 3508 速度 PID。6020 保持角度环与速度环串级 PID。

### 7.3 底盘行为状态机

文件：`User/APP/chassis_behaviour.c`

```text
遥控器底盘拨杆 / 键盘 X C Shift / Q
  -> chassis_behaviour_mode_set()
  -> chassis_behaviour_mode
  -> chassis_mode
  -> chassis_behaviour_control_set()
  -> vx_set / vy_set / wz_set
```

行为与输出：

| 行为 | 控制模式 | 输出定义 |
|---|---|---|
| `CHASSIS_NO_MOVE` | `CHASSIS_VECTOR_NO_MOVE` | `vx_set = 0`，`vy_set = 0`，`wz_set = 0` |
| `CHASSIS_FOLLOW_GIMBAL_YAW` | `CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW` | 遥控器/键盘给 vx/vy，遥控器 yaw 通道给 wz |
| `CHASSIS_SPIN` | `CHASSIS_VECTOR_SPIN` | 遥控器/键盘给 vx/vy，`CHASSIS_SPIN_SPEED` 给 wz |
| `CHASSIS_RETURN` | `CHASSIS_VECTOR_RETURN` | 回正逻辑保留，当前触发逻辑处于注释状态 |

`Q` 键用于超级电容模式切换：

```text
KEY_PRESSED_OFFSET_Q 上升沿
  -> super_cap_mode >= SUPER_CAP_PREPARED
  -> SUPER_CAP_PREPARED <-> SUPER_CAP_USING
```

### 7.4 底盘输入速度生成链路

```text
chassis_rc_to_control_vector()
  -> rc_deadband_limit(CHASSIS_X_CHANNEL)
  -> rc_deadband_limit(CHASSIS_Y_CHANNEL)
  -> 遥控器通道 * CHASSIS_VX_RC_SEN / CHASSIS_VY_RC_SEN
  -> WASD 键按住时按计数斜坡叠加速度
  -> first_order_filter_cali(vx/vy)
  -> 输出 vx_set / vy_set
```

`first_order_filter_cali()` 对 vx/vy 目标做一阶滤波，降低速度设定突变。`chassis_set_contorl()` 会根据当前模式把速度向量旋转到需要的坐标系，并用 `vx_max_speed`、`vy_max_speed` 限幅。

坐标旋转链路：

```text
vx_set / vy_set
  -> vector_rotate(gimbal_radian_of_ecd + 模式 offset)
  -> fp32_constrain()
  -> chassis_move.vx_set / chassis_move.vy_set
```

`gimbal_radian_of_ecd` 是底盘坐标旋转使用的云台 yaw 相对角字段。当前代码声明了该字段并在 `vector_rotate()`、回正 PID、静止保角逻辑里使用它；排查底盘跟随云台方向时，应先确认 `chassis_feedback_update()` 或等效入口已经把云台 yaw 相对角写入该字段。

### 7.5 舵轮运动学链路

文件：`User/APP/chassis_calculate.c`

```text
chassis_control_loop()
  -> chas_inv_cal(vx_set, vy_set, wz_set)
  -> 计算每个轮子的 vx_total / vy_total
  -> wheel_speed[i] = sqrt(vx_total^2 + vy_total^2)
  -> wheel_angle[i] = atan2(vy_total, vx_total) - wheel_angle_offset.now[i]
  -> smooth_control()
  -> chassis_6020[i].angle_set = wheel_angle[i]
  -> chassis_3508[i].speed_set = wheel_speed[i]
```

`smooth_control()` 执行就近转向策略：当目标角与当前 6020 角差超过阈值时，舵向目标加减 `PI`，驱动轮速度取反。这样能减少舵向大角度转动时间。

### 7.6 3508 驱动轮功控链路

文件：`User/APP/chassis_task.c`、`User/APP/chassis_power_control.c`、`swerve_power_core_only/application/3508_motor_change_log.md` 来源说明

完整链路：

```text
chas_inv_cal()
  -> chassis_3508[i].speed_set
  -> Model_Based_Control(i, speed_set, speed)
  -> model_3508_out[i]
  -> chassis_power_control()
  -> chassis_3508[i].give_current
  -> CAN_cmd_CHAS_3508()
  -> FDCAN1 ID 0x200
  -> 3508 电调
```

`Model_Based_Control()` 的计算链路：

```text
speed_set - ref_speed
  -> error_v
  -> accel_target = error_v / CONTROL_PERIOD_MODEL
  -> jerk 限制更新 model_accel[i]
  -> CHASSIS_MAX_ACCEL 限幅
  -> F_traction = ROBOT_MASS / 4 * model_accel[i]
  -> 低速线性摩擦补偿或常值摩擦补偿
  -> 小误差 hold P 补偿
  -> Torque_output = F_total * Wheel_Radius / CHASSIS_EFFICIENCY
  -> 连续扭矩限幅
  -> Current_A = Torque_output / M3508_TORQUE_CONSTANT
  -> 20A 对应 16384 的电调命令映射
  -> M3505_MOTOR_SPEED_PID_MAX_OUT 限幅
  -> model_3508_out[i]
```

当前底盘功率函数入口：

```text
chassis_power_control()
  -> pm01_access_poll()
  -> 根据 robot_status.chassis_power_limit 更新 PowerLimit.set_power
  -> 当前无超级电容分支直接赋值：
       chassis_3508[i].give_current = (int16_t)model_3508_out[i]
       chassis_6020[i].give_current = (int16_t)chas_6020_speed_pid[i].out
```

功率预测函数保留链路：

```text
Current_RestraintRelation_Calc()
  -> now_motorspeed[i] = chassis_3508[i].feedback speed_rpm
  -> set_motorcurrent[i] = model_3508_out[i]
  -> set_6020_current[i] = chas_6020_speed_pid[i].out

Predict_Power()
  -> 预留 6020 功率
  -> 计算 3508 原始功率 P_origin
  -> P_origin 超过 available_power 时求 K_Reduction
  -> 3508 give_current = model_3508_out[i] * K_Reduction
  -> 6020 give_current = chas_6020_speed_pid[i].out
```

重点：3508 的控制量从 `speed_set` 到 `model_3508_out` 再到 `give_current`，没有经过 3508 速度 PID；功率预测链路读取的 3508 输入也是 `model_3508_out`。

### 7.7 6020 舵向闭环链路

```text
chas_inv_cal()
  -> chassis_6020[i].angle_set
  -> PID_Calc_Jump()
  -> angle_error 过 PI 时调整 angle_set
  -> chas_6020_angle_pid[i]
  -> chas_6020_speed_pid[i]
  -> chassis_power_control()
  -> chassis_6020[i].give_current
  -> CAN_cmd_CHAS_6020()
  -> FDCAN2 ID 0x1FE
  -> 6020 电调
```

6020 的反馈来自 `CHASSIS_MOTOR_MEASURE[4..7].ecd`，角度换算为：

```text
angle = rad_format(ecd / GM6020_Angle_Ratio)
```

6020 的速度环反馈来自 `CHASSIS_MOTOR_MEASURE[4..7].speed_rpm`，速度 PID 输出直接作为最终 6020 电流来源。

### 7.8 底盘安全输出链路

```text
detect_task()
  -> toe_is_error(CHASSIS_MOTORx_TOE)
  -> toe_is_error(DBUS_TOE)
  -> chassis_task()
```

底盘任务启动后先等待 8 个底盘电机和 DBUS 在线。运行中只要 8 个电机不是全部离线，就继续允许 CAN 包发送；当 DBUS 离线时，底盘发送：

```text
CAN_cmd_CHAS_6020(0, 0, 0, 0)
CAN_cmd_CHAS_3508(0, 0, 0, 0)
```

这个链路让遥控器丢失直接切断底盘电流输出。

## 8. 自瞄控制链路

文件：`User/APP/auto_aim.c`、`User/APP/yaw_pitch_direct.c`

### 8.1 自瞄任务链路

```text
auto_aim_task()
  -> auto_aim_init()
  -> 循环：
       auto_aim_set()
    -> auto_aim_feedback_update()
    -> auto_aim_control_tick_internal()
    -> osDelay(AUTO_AIM_TIME)
```

`auto_aim_apply_delta_udeg()` 是外部通信写入自瞄误差的入口。它把 micro-degree 转换为 rad，并写入 `s_auto_aim_ctrl.yaw_axis.err_rad` 与 `s_auto_aim_ctrl.pitch_axis.err_rad`。

### 8.2 自瞄误差到云台目标链路

```text
auto_aim_apply_delta_udeg()
  -> dyaw_udeg / dpitch_udeg 转 rad
  -> s_auto_aim_ctrl.*.err_rad
  -> auto_aim_control_tick_internal()
  -> 误差 PD 成角速度
  -> 角速度限幅
  -> 加速度限幅
  -> dt 积分为 yaw_delta / pitch_delta
  -> 五点中值滤波
  -> aim.receive.yaw / aim.receive.pitch 用于状态观察
  -> yaw_pitch_direct.c 通过 auto_aim_get_yaw_err_rad() / auto_aim_get_pitch_err_rad() 读取 err_rad
  -> gimbal_set_control()
  -> yaw/pitch 目标角规划
```

自瞄只提供目标修正量和状态，不直接下发电机。当前云台目标规划读取的是 `err_rad`，`aim.receive.yaw` 与 `aim.receive.pitch` 是自瞄任务内部计算后的增量状态。

## 9. 在线检测链路

文件：`User/APP/detect_task.c`

```text
detect_hook(toe)
  -> new_time = xTaskGetTickCount()
  -> error_exist = 0
  -> frequency = configTICK_RATE_HZ / 时间差

detect_task()
  -> 周期扫描 error_list
  -> now - new_time > set_offline_time
  -> is_lost = 1
  -> error_exist = 1

toe_is_error(toe)
  -> 返回 error_exist
```

当前显式接入检测的对象：

```text
DBUS_TOE：由 rc_ctrl.last_fdb 刷新
CHASSIS_MOTOR1_TOE ~ CHASSIS_MOTOR4_TOE：FDCAN1 0x201~0x204 刷新
CHASSIS_MOTOR5_TOE ~ CHASSIS_MOTOR8_TOE：FDCAN2 0x205~0x208 刷新
```

检测结果当前直接影响底盘启动等待、遥控器离线零电流输出、底盘电机在线发送条件。

## 10. 通信与调试链路

### 10.1 VOFA 调试链路

文件：`User/Devices/vofa.c`、`User/APP/gimbal_task.c`

```text
gimbal_task()
  -> VOFA_Send6(
       fric1.speed_rpm,
       fric2.speed_rpm,
       fric3.speed_rpm,
       fric1.speed_mps,
       fric2.speed_mps,
       fric3.speed_mps
     )
```

VOFA 当前输出三路摩擦轮转速和线速度，用于观察发射闭环。

### 10.2 USB/通信协议链路

文件：`User/Communication/example/device/comm_app.c`、`User/Communication/channel/*`

```text
comm_app_start()
  -> 通信任务
  -> gimbal/camera/time_sync channel
  -> auto_aim_apply_delta_udeg()
  -> 自瞄误差进入 auto_aim 链路
```

通信模块负责把上位机或图传侧数据转换为工程内部回调。自瞄误差最终通过 `auto_aim_apply_delta_udeg()` 进入云台控制链路。

### 10.3 服务任务链路

文件：`User/APP/service_task.c`、`User/APP/Safewarning.c`

```text
service_task()
  -> Beep_Init()
  -> hwt_imu_init()
  -> Beep_Play(BEEP_POWER_ON)
  -> 循环：
       ws2812_task()
    -> Beep_Task()
```

服务任务不参与高速闭环，只负责低频状态服务。`hwt_imu_init()` 在这里清空 HWT101/HWT906 的解析器和导出数组。

## 11. 电机与 CAN ID 总表

| 对象 | 反馈总线 | 反馈 ID | 反馈数组 | 下发总线 | 下发 ID | 下发函数 |
|---|---|---|---|---|---|---|
| 底盘 3508 驱动轮 1~4 | FDCAN1 | `0x201~0x204` | `CHASSIS_MOTOR_MEASURE[0..3]` | FDCAN1 | `0x200` | `CAN_cmd_CHAS_3508()` |
| 底盘 6020 舵向 1~4 | FDCAN2 | `0x205~0x208` | `CHASSIS_MOTOR_MEASURE[4..7]` | FDCAN2 | `0x1FE` | `CAN_cmd_CHAS_6020()` |
| 云台 yaw/pitch MIT | FDCAN2 | `0x51` / `0x52` | `MIT_MOTOR_MEASURE[0..1]` | FDCAN2 | `0x01` / `0x02` | `CAN_cmd_MIT()` |
| 发射摩擦轮 1~3 | FDCAN2 | `0x201~0x203` | `DJI_MOTOR_MEASURE[0..2]` | FDCAN2 | `0x200` | `shoot_task_send_friction_current()` |
| 发射拨弹反馈 | FDCAN2 | `0x204` | `DJI_MOTOR_MEASURE[3]` | 当前发射发送帧第 4 路为 0 | `0x200` | `shoot_task_send_friction_current()` |
| PM01 | FDCAN1 | `0x600~0x603` / `0x610~0x613` | `pm01_od` | FDCAN1 | `0x600~0x603` | `pm01_*_set()` |

FDCAN1 和 FDCAN2 是两条总线，因此 FDCAN1 的底盘 3508 下发 `0x200` 与 FDCAN2 的发射摩擦轮下发 `0x200` 可以同时存在。云台 MIT 反馈索引为 yaw `0`、pitch `1`，对应 `MIT_MOTOR_MEASURE[0]` 与 `MIT_MOTOR_MEASURE[1]`。

## 12. 参数入口

云台参数主要在 `User/APP/project_config.h`：

```text
YAW_GYRO_ABSOLUTE_PID_*
YAW_ENCODE_RELATIVE_PID_*
PITCH_GYRO_ABSOLUTE_PID_*
PITCH_ENCODE_RELATIVE_PID_*
YAW_INERTIA_KGM2
PITCH_INERTIA_KGM2
PITCH_GRAVITY_COMP_*
YAW_MAX_RELATIVE_ANGLE / YAW_MIN_RELATIVE_ANGLE
PITCH_MAX_RELATIVE_ANGLE / PITCH_MIN_RELATIVE_ANGLE
DM_YAW_CAN_ID / DM_PIT_CAN_ID
DM_YAW_MASTER_ID / DM_PIT_MASTER_ID
```

底盘参数主要在 `User/APP/chassis_task.h` 与 `User/APP/robot_param.h`：

```text
CHASSIS_VX_RC_SEN / CHASSIS_VY_RC_SEN / CHASSIS_WZ_RC_SEN
NORMAL_MAX_CHASSIS_SPEED_X / NORMAL_MAX_CHASSIS_SPEED_Y
HALF_LENGTH / HALF_WIDTH
CHASSIS_6020_INIT_ANGLE_*
CHASSIS_FOLLOW_GIMBAL_YAW_OFFSET
CHASSIS_SPIN_OFFSET
ROBOT_MASS
M3508_TORQUE_CONSTANT
CHASSIS_EFFICIENCY
CHASSIS_MAX_ACCEL / CHASSIS_MAX_JERK
FRICTION_* / SPEED_HOLD_*
GM6020_MOTOR_ANGLE_PID_*
GM6020_MOTOR_SPEED_PID_*
```

发射参数主要在 `User/APP/shoot_task.h`：

```text
SHOOT_FRIC_TARGET_SPEED_RPM
SHOOT_FRIC_MAX_CURRENT
SHOOT_FRIC*_DIRECTION
SHOOT_FRIC*_B0
SHOOT_FRIC*_RESPONSE_TIME_S
SHOOT_FRIC*_OBSERVER_RATIO
SHOOT_FRIC*_OUTPUT_RATE_LIMIT
SHOOT_FRIC*_FF_*
SHOOT_FRIC_FDB_TIMEOUT
SHOOT_FRIC_TEMP_LIMIT
```

功率与 PM01 参数主要在 `User/APP/chassis_power_control.h` 与 `User/APP/pm01_api.c`：

```text
SET_POWER_VALUE
PowerLimit.k_1 / k_2 / a
PowerLimit.set_power
g_cmd_set
g_power_set
g_vout_set
g_iout_set
ROBOT_CAP / Cap_on
```

## 13. 当前控制链路重点

### 13.1 底盘 3508 当前核心路径

```text
遥控器/键盘输入
  -> vx_set / vy_set / wz_set
  -> chas_inv_cal()
  -> speed_set
  -> Model_Based_Control()
  -> model_3508_out
  -> chassis_power_control()
  -> give_current
  -> CAN_cmd_CHAS_3508()
```

3508 控制的核心变量是 `model_3508_out[4]`。排查 3508 输出时，应沿 `speed_set`、`speed`、`model_accel`、`model_3508_out`、`give_current`、FDCAN1 `0x200` 逐级查看。

### 13.2 底盘 6020 当前核心路径

```text
vx_set / vy_set / wz_set
  -> chas_inv_cal()
  -> angle_set
  -> angle PID
  -> speed PID
  -> give_current
  -> CAN_cmd_CHAS_6020()
```

6020 控制的核心变量是 `chassis_6020[i].angle_set`、`chassis_6020[i].angle`、`chas_6020_angle_pid[i].out`、`chas_6020_speed_pid[i].out`。

### 13.3 云台当前核心路径

```text
HWT906 / HWT101 / MIT 反馈 + 遥控器/自瞄输入
  -> gimbal_feedback_update()
  -> gimbal_set_control()
  -> gimbal_control_loop()
  -> gravity_comp_execute()
  -> CAN_cmd_MIT()
```

云台控制的核心变量是 `absolute_angle_set`、`relative_angle_set`、`absolute_angle`、`relative_angle`、`gyro`、`pid_torque`、`ff_torque`、`given_current`。

### 13.4 发射当前核心路径

```text
R/G/拨杆 + 云台停射联锁
  -> shoot_task_set_mode()
  -> DJI_MOTOR_MEASURE[0..2]
  -> ADRC_Calc()
  -> 掉速前馈
  -> current_a / current_mA / ESC cmd
  -> FDCAN2 0x200
```

发射控制的核心变量是 `speed_rpm`、`speed_set_rpm`、`give_current_a`、`give_current`、`ff_ticks`、`ff_cooldown_ticks`。

## 14. 三轮审核记录

### 14.1 第一轮：从控制调试视角审核

补充结果：为每条高速闭环增加了“核心变量”说明，修正了 3508 变更记录路径，补充了 `gimbal_radian_of_ecd` 对底盘坐标旋转的影响，底盘 3508、底盘 6020、云台、发射都能按变量名逐级排查。

### 14.2 第二轮：从硬件接线视角审核

补充结果：增加了 FDCAN1/FDCAN2 的接收 ID、发送 ID、反馈数组和下发函数总表，补齐了 MIT 的具体 ID、发射拨弹反馈、两条总线共用 `0x200` 的关系，明确 3508 与 PM01 在 FDCAN1，云台 MIT、发射摩擦轮、底盘 6020 在 FDCAN2。

### 14.3 第三轮：从移植完整性视角审核

补充结果：增加了底盘移植状态、依赖文件、复用 BSP/Devices/Algorithm 的关系、Keil 工程配置、`ARM_MATH_CM7` 宏作用；标出了裁判系统解析函数已存在、USART1 当前未分发到解析器；标出了 3508 已从速度 PID 输出切到 `model_3508_out`，功率预测保留函数也读取 `model_3508_out`。
