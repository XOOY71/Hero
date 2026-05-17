# Bug 修复记录

## 1. Pitch 轴斜坡目标跟踪呈阶梯型并滞后

**修复时间**：2026-05-17 04:58:47 +08:00

**问题现象**

pitch 轴目标位置为连续斜坡时，实际位置没有连续跟随，而是呈阶梯型运动并带有明显滞后。VOFA 曲线中，总输出力矩呈波浪型；去掉线型前馈后，PID 输出仍然表现为波浪型。惯量前馈在斜坡匀速段贡献很小，运动主要由反馈输出推动。

**根因定位**

pitch 轴位置反馈使用 MIT 电机编码器相对角，速度反馈使用 IMU gyro Y。位置闭环和速度反馈来自不同测量链路，经过不同传感器、安装位置和机械弹性路径，导致相位和噪声特性不一致。控制器计算 `speed_error = ref_vel - gyro` 时，这个不一致会直接进入 D 项输出，形成周期性的 PID 力矩波动。

原前馈只包含惯量加速度项：

```c
ff_torque = J * ref_accel;
```

目标位置为匀速斜坡时，参考加速度 `ref_accel` 只在起停阶段明显，匀速段接近 0，所以前馈在主要运动阶段无法提供持续力矩。

**修复方式**

pitch 轴速度反馈改为由同一个位置反馈源生成：使用 `relative_angle` 做差分，并经过低通滤波得到 `relative_speed`。pitch 速度误差改为：

```c
speed_error = ref_vel - relative_speed;
```

pitch 前馈从单一惯量前馈扩展为速度前馈加惯量前馈：

```c
ff_torque = PITCH_VELOCITY_FF_GAIN * ref_vel + J * ref_accel;
```

这样斜坡匀速段由 `PITCH_VELOCITY_FF_GAIN * ref_vel` 提供持续前馈力矩，起停阶段由 `J * ref_accel` 补偿加速度需求。

**涉及文件**

- `User/APP/gimbal_task.c`
- `User/APP/gimbal_task.h`
- `User/APP/yaw_pitch_direct.c`
- `User/APP/project_config.h`

**验证结果**

VOFA 观察结果显示，pitch 实际位置跟踪恢复连续，PID 输出波浪明显收敛，前馈在斜坡匀速段持续参与输出。
