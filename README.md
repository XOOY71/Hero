# Hero 业务代码框架说明

本文只说明本工程的业务电控代码组织和控制链路，不展开芯片、IDE、CubeMX 外设初始化等环境内容。

## 总体业务结构

业务代码主要分为四层：

```text
User/APP        任务与业务状态机：云台、发射、服务任务
User/BSP        板级通信封装：FDCAN、UART、遥控器解析
User/Devices    设备驱动与数据解析：IMU、VOFA、WS2812
User/Algorithm  控制算法：PID、ADRC、重力补偿、滤波
```

核心运行链路是：

```text
遥控器 / IMU / CAN 反馈
  -> BSP 和 Devices 解析为全局反馈数据
  -> APP 层状态机决定模式和目标值
  -> Algorithm 层计算控制输出
  -> BSP 层通过 CAN/UART 下发或调试输出
```

## APP 层

### 云台主任务：`gimbal_task`

文件：

- `User/APP/gimbal_task.c`
- `User/APP/gimbal_task.h`

`gimbalTask` 是业务主循环，周期由 `GIMBAL_CONTROL_TIME` 配置，当前为 1 ms。它负责 yaw/pitch 云台闭环，并在同一个周期内调用发射控制循环。

主循环顺序：

```text
gimbal_set_mode()
  -> gimbal_feedback_update()
  -> gimbal_mode_change_control_transit()
  -> gimbal_set_control()
  -> gimbal_control_loop()
  -> gimbal_pitch_soft_limit_output()
  -> gravity_comp_execute()
  -> gimbal_send_cmd()
  -> shoot_task_loop()
  -> VOFA_Send6()
```

这条链路体现了工程的主要设计：先确定行为模式，再刷新反馈，再平滑切换目标，最后计算输出并下发电机命令。

### 云台行为状态机：`gimbal_behaviour`

文件：

- `User/APP/gimbal_behaviour.c`
- `User/APP/gimbal_behaviour.h`

该模块负责把遥控器拨杆、键鼠输入和当前云台状态转换成高层行为。

主要行为：

- `GIMBAL_ZERO_FORCE`：零力输出，不主动控制。
- `GIMBAL_INIT`：从零力切入控制时先归中。
- `GIMBAL_CALI`：校准流程预留。
- `GIMBAL_ABSOLUTE_ANGLE`：绝对角控制。
- `GIMBAL_RELATIVE_ANGLE`：相对角控制。
- `GIMBAL_MOTIONLESS`：静止/停止状态。
- `GIMBAL_SPIN`：小陀螺/旋转状态。

状态机还会给其他模块提供约束接口：

- `gimbal_cmd_to_chassis_stop()`：云台当前状态是否要求底盘停止。
- `gimbal_cmd_to_shoot_stop()`：云台当前状态是否要求发射停止。

发射模块会调用 `gimbal_cmd_to_shoot_stop()`，所以云台初始化、校准、零力或静止时，摩擦轮会被强制停机。

### 云台平台适配：`yaw_pitch_direct`

文件：

- `User/APP/yaw_pitch_direct.c`
- `User/APP/yaw_pitch_direct.h`

该模块把通用云台控制框架接到当前硬件反馈和执行器上，是云台业务的关键适配层。

它负责：

- 初始化 `gimbal_control_t`。
- 绑定遥控器、IMU、CAN 电机反馈。
- 初始化 yaw/pitch PID。
- 让达妙 yaw/pitch 电机进入 MIT 模式并使能。
- 计算 yaw 相对底盘角、pitch 相对零位角。
- 根据行为层输出更新目标角。
- 叠加惯量前馈轨迹。
- 将最终 yaw/pitch 力矩通过 FDCAN2 下发。

云台反馈定义：

```text
yaw 绝对角 = HWT906 连续 yaw
yaw 相对角 = HWT906 连续 yaw - HWT101 底盘连续 yaw - 上电零偏
pitch 角度 = 达妙 pitch MIT 位置反馈 - 上电零偏
yaw 角速度 = HWT906 gyro.z
pitch 角速度 = HWT906 gyro.y
```

云台输出定义：

```text
given_current
  -> 限幅到 MIT 力矩范围 T_MIN ~ T_MAX
  -> CAN_cmd_MIT(&hfdcan2, DM_YAW_CAN_ID, ...)
  -> CAN_cmd_MIT(&hfdcan2, DM_PIT_CAN_ID, ...)
```

### 发射控制：`shoot_task`

文件：

- `User/APP/shoot_task.c`
- `User/APP/shoot_task.h`

发射模块当前控制三路摩擦轮，不单独创建 FreeRTOS 任务，而是在云台 1 ms 主循环里由 `shoot_task_loop()` 调用。

工作模式：

- `SHOOT_TASK_STOP`：停机，清零目标转速、输出电流和前馈状态。
- `SHOOT_TASK_READY_FRIC`：摩擦轮闭环稳速。

启停输入：

- `R` 键上升沿：开启摩擦轮。
- `G` 键上升沿：关闭摩擦轮。
- 发射拨杆下档：强制开启摩擦轮。
- 发射拨杆中档：强制关闭摩擦轮。
- 云台要求停射时：强制进入 `SHOOT_TASK_STOP`。

摩擦轮控制链路：

```text
DJI_MOTOR_MEASURE[n].speed_rpm
  -> 按安装方向归一化
  -> ADRC_Calc()
  -> 电流限幅
  -> 掉速前馈补偿
  -> CAN ID 0x200 三路电流帧
```

保护逻辑：

- 反馈超时超过 `SHOOT_FRIC_FDB_TIMEOUT` 停机。
- 温度达到 `SHOOT_FRIC_TEMP_LIMIT` 停机。
- 电流输出受 `SHOOT_FRIC_MAX_CURRENT` 限制。
- 单发掉速满足阈值时，按 `SHOOT_FRIC*_FF_*` 参数短时叠加固定前馈电流。

### 服务任务：`service_task`

文件：

- `User/APP/service_task.c`
- `User/APP/service_task.h`

服务任务不参与高速闭环，主要处理低优先级状态服务：

- `Beep_Init()` / `Beep_Task()`：蜂鸣器提示音状态机。
- `hwt_imu_init()`：初始化 HWT IMU 解析数据结构。
- `ws2812_task()`：WS2812 状态灯刷新。

### 状态提示：`Safewarning`

文件：

- `User/APP/Safewarning.c`
- `User/APP/Safewarning.h`

该模块实现蜂鸣器曲谱、PWM 频率设置和 WS2812 简单灯效。业务上主要用于开机提示、错误提示、成功提示和状态显示。

### 全局配置：`project_config`

文件：

- `User/APP/project_config.h`

这是业务参数集中配置文件，包含：

- 云台 PID 参数。
- yaw/pitch 惯量前馈参数。
- pitch 重力补偿参数。
- 遥控器通道和灵敏度。
- 云台软件限位。
- CAN 电机 ID。
- MIT 协议量程。

## BSP 层

### FDCAN：`bsp_fdcan`

文件：

- `User/BSP/bsp_fdcan.c`
- `User/BSP/bsp_fdcan.h`

该模块负责 CAN 总线收发和电机反馈解析。

主要数据结构：

- `MIT_MOTOR_MEASURE[]`：达妙 MIT 电机反馈，包括位置、速度、力矩和温度。
- `DJI_MOTOR_MEASURE[]`：DJI 电机反馈，包括编码器、转速、电流和温度。
- `can_error_status`：CAN 错误状态。

接收解析：

```text
FDCAN2 RX
  -> DM_YAW_MASTER_ID / DM_PIT_MASTER_ID
  -> MITFdbData()
  -> MIT_MOTOR_MEASURE[]

FDCAN2 RX
  -> CAN_FRIC1_ID / CAN_FRIC2_ID / CAN_FRIC3_ID / CAN_STRUM_ID
  -> get_motor_measure()
  -> DJI_MOTOR_MEASURE[]
```

命令发送：

- `CAN_cmd_MIT()`：打包达妙 MIT 协议力矩帧。
- `canx_send_data()`：通用 FDCAN 发送封装。
- `Motor_MIT_MODE()`：切入 MIT 模式。
- `Motor_ENABLE()`：电机使能。

### UART 与数据分发：`bsp_usart`

文件：

- `User/BSP/bsp_usart.c`
- `User/BSP/bsp_usart.h`

该模块主要处理 UART 空闲中断 DMA 回调，并把不同串口的数据分发给对应业务解析器。

```text
UART5
  -> sbus_to_rc()
  -> rc_ctrl

UART7
  -> hwt101_rx_parse()
  -> HWT101 底盘 yaw 数据

USART10
  -> hwt906_rx_parse()
  -> HWT906 云台姿态 / 角速度 / 加速度
```

UART 错误回调中会清空对应接收缓冲并重新开启 DMA 接收，用于保证热插拔和异常帧后的恢复能力。

### 遥控器解析：`remote_control`

文件：

- `User/BSP/remote_control.c`
- `User/BSP/remote_control.h`

该模块解析类 SBUS 数据，将遥控器通道、拨杆、鼠标和键盘映射到统一的 `RC_ctrl_t rc_ctrl`。

业务模块不直接解析原始 SBUS 帧，而是通过：

```text
get_remote_control_point()
```

获取只读遥控器数据指针。

## Devices 层

### HWT IMU：`hwt_imu`

文件：

- `User/Devices/hwt_imu.c`
- `User/Devices/hwt_imu.h`

该模块解析 HWT 系列 IMU 的 `0x55` 数据帧，并导出连续 yaw、pitch、角速度和加速度。

HWT101 用途：

- 作为底盘 yaw 参考。
- 导出 `hwt101_get_yaw_total_rad()`。

HWT906 用途：

- 作为云台姿态和角速度反馈。
- 导出 `get_INS_angle_point()`、`get_gyro_data_point()`、`get_accel_data_point()`。

云台适配层通过这些接口读取姿态，不直接接触串口字节流。

### VOFA 调试：`vofa`

文件：

- `User/Devices/vofa.c`
- `User/Devices/vofa.h`

当前云台主循环中使用 `VOFA_Send6()` 输出三路摩擦轮转速和前馈电流，方便观察发射速度环。

### WS2812：`ws2812`

文件：

- `User/Devices/ws2812.c`
- `User/Devices/ws2812.h`

提供 WS2812 灯带控制接口，由 `Safewarning` / `service_task` 调用。

## Algorithm 层

### PID：`pid`

文件：

- `User/Algorithm/pid.c`
- `User/Algorithm/pid.h`

提供位置式和增量式 PID。云台 yaw/pitch 角度闭环通过 `gimbal_pid_init()`、`gimbal_pid_calc()` 间接调用该模块。

### ADRC：`adrc`

文件：

- `User/Algorithm/adrc.c`
- `User/Algorithm/adrc.h`

提供一阶 ADRC 控制器和 ESO 扰动观测器。发射模块的三路摩擦轮速度环使用 ADRC：

```text
ADRC_init()
ADRC_reset() / ADRC_hot_reset()
ADRC_Calc()
```

`shoot_task` 在摩擦轮从停止切入运行时使用热启动复位，减少闭环接管瞬间的电流突变。

### 重力补偿：`gravity_comp`

文件：

- `User/Algorithm/gravity_comp.c`
- `User/Algorithm/gravity_comp.h`

该模块根据 pitch 角度、等效质量、质心前向/上向距离计算重力补偿力矩。云台主循环在 PID 和惯量前馈之后调用：

```text
gravity_comp_execute(&gimbal_control)
```

补偿结果会叠加到 pitch 输出，并受 `PITCH_GRAVITY_COMP_OUTPUT_LIMIT` 限制。

## 主要业务数据流

### 云台闭环数据流

```text
遥控器 / 键鼠
  -> gimbal_behaviour_control_set()
  -> yaw/pitch 目标增量

HWT906 + HWT101 + MIT 电机反馈
  -> gimbal_feedback_update()
  -> absolute_angle / relative_angle / gyro

目标值 + 反馈值
  -> PID + 惯量前馈 + pitch 重力补偿
  -> given_current
  -> CAN_cmd_MIT()
```

### 发射闭环数据流

```text
遥控器按键 / 拨杆 + 云台停射约束
  -> shoot_task_set_mode()

DJI 摩擦轮反馈
  -> shoot_task_update_feedback()
  -> ADRC 速度闭环
  -> 掉速前馈
  -> CAN 0x200 电流输出
```

## 主要业务调参入口

云台相关：

- `YAW_GYRO_ABSOLUTE_PID_*`
- `PITCH_GYRO_ABSOLUTE_PID_*`
- `YAW_ENCODE_RELATIVE_PID_*`
- `PITCH_ENCODE_RELATIVE_PID_*`
- `YAW_INERTIA_KGM2`
- `PITCH_INERTIA_KGM2`
- `PITCH_GRAVITY_COMP_*`
- `YAW_MAX_RELATIVE_ANGLE`
- `PITCH_MAX_RELATIVE_ANGLE`
- `PITCH_MIN_RELATIVE_ANGLE`

遥控器相关：

- `YAW_CHANNEL`
- `PITCH_CHANNEL`
- `GIMBAL_MODE_CHANNEL`
- `YAW_RC_SEN`
- `PITCH_RC_SEN`
- `YAW_MOUSE_SEN`
- `PITCH_MOUSE_SEN`
- `RC_DEADBAND`

发射相关：

- `SHOOT_FRIC_TARGET_SPEED_RPM`
- `SHOOT_FRIC_MAX_CURRENT`
- `SHOOT_FRIC*_B0`
- `SHOOT_FRIC*_RESPONSE_TIME_S`
- `SHOOT_FRIC*_OBSERVER_RATIO`
- `SHOOT_FRIC*_OUTPUT_RATE_LIMIT`
- `SHOOT_FRIC*_FF_*`

CAN ID 相关：

- `DM_YAW_CAN_ID`
- `DM_PIT_CAN_ID`
- `DM_YAW_MASTER_ID`
- `DM_PIT_MASTER_ID`
- `CAN_FRIC1_ID`
- `CAN_FRIC2_ID`
- `CAN_FRIC3_ID`
- `CAN_STRUM_ID`

## 维护建议

1. 新增高速闭环业务时，优先明确它是否应挂在 `gimbalTask` 的 1 ms 主循环中；低频状态服务不要放进主控制链路。
2. 反馈解析应放在 BSP/Devices 层，APP 层只读取结构化结果。
3. 控制算法应放在 Algorithm 层，APP 层只负责组织状态机、目标值和输出限幅。
4. 新增参数优先集中到 `project_config.h` 或对应业务头文件，避免散落在 `.c` 文件中。
5. 修改电机 ID、方向、限位和传感器坐标定义后，应同步检查云台反馈定义和输出极性。
