# 3508 电机相关改动整理（当前代码）

本文档基于当前工程代码状态整理，仅聚焦 `3508` 相关改动与影响链路。

## 1. 改动总览

- `3508` 主控制已从传统速度环 `PID_calc` 切换到 `Model_Based_Control()`。
- 新增独立输出通道 `model_3508_out[4]`，不再复用 `chas_3508_pid[i].out` 传递控制量。
- 功率控制侧已改为读取 `model_3508_out[4]` 作为 `3508` 目标电流输入。
- `6020` 角度环/速度环 PID 保持原逻辑不变。

## 2. 参数层改动（`chassis_task.h`）

文件：`application/chassis_task.h`

### 2.1 物理模型参数

- `ROBOT_MASS`：机器人质量（kg）
- `M3508_TORQUE_CONSTANT`：3508 力矩常数（Nm/A）
- `M3508_REDUCTION_RATIO`：3508 减速比
- `CHASSIS_EFFICIENCY`：机械效率
- `CONTROL_PERIOD_MODEL`：模型控制周期
- `RESISTANCE_ACCUM_KI`：阻力累积增益
- `RESISTANCE_ACCUM_MAX`：阻力累积限幅

### 2.2 新增 3508 物理模型输出

- 在 `chassis_move_t` 中新增：
  - `fp32 model_3508_out[4]`

用途：
- 专门承载 `3508` 物理模型的电流控制值，供功率控制与电流下发使用。

## 3. 初始化与反馈改动（`chassis_task.c`）

文件：`application/chassis_task.c`

- `chassis_init()` 中移除了 `3508` 的 `PID_init`。
- 在初始化循环中将 `model_3508_out[i]` 清零。
- `chassis_feedback_update()` 中 `3508` 加速度改为速度差分计算，不再使用 PID 微分缓存。

## 4. 控制计算改动（`chassis_task.c`）

文件：`application/chassis_task.c`

### 4.1 模型控制函数

- 函数：`Model_Based_Control(uint8_t motor_idx, fp32 set_speed, fp32 ref_speed)`

核心过程：
- 速度误差 `error_v = set_speed - ref_speed`
- 由 `a = dv/dt` 估计加速度
- 由 `F = m*a` 计算牵引力
- 叠加阻力累积项（带限幅）
- 力换算为电机扭矩，再换算为电流
- 电流映射到 3508 控制量（20A -> 16384）
- 最后进行输出限幅

### 4.2 主控制写入目标

在 `PID_Calc_Jump()` 中：
- 当前实现为  
  `model_3508_out[i] = Model_Based_Control(...)`
- 原 3508 `PID_calc(...)` 路径不再参与控制输出。

## 5. 功率控制链路改动（`chassis_power_control.c/.h`）

文件：`application/chassis_power_control.c`、`application/chassis_power_control.h`

### 5.1 3508 输入来源

- `Current_RestraintRelation_Calc()` 读取：
  - `PowerLimit_Cur->set_motorcurrent[i] = chassis_move.model_3508_out[i]`

### 5.2 最终下发来源

- `chassis_power_control()` 无超级电容分支中：
  - `chassis_3508[i].give_current = (int16_t)model_3508_out[i]`

说明：
- 功率控制侧已统一使用物理模型输出，不再读取 3508 PID 输出字段。

## 6. 当前版本下的 3508 链路关系

1. 逆解算得到 `speed_set`  
2. `Model_Based_Control()` 计算 `model_3508_out`  
3. 功率控制读取 `model_3508_out` 做预测与约束  
4. `CAN_cmd_CHAS_3508(...)` 发送最终 `give_current`

## 7. 仍需关注项

- `Predict_Power()` 中对 `P_in` 的改善系数计算仍有除零风险（`P_in == 0`）。
- 功率控制中的超级电容状态机代码有部分注释路径，后续建议按最终方案统一清理。
