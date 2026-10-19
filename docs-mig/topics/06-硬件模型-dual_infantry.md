# 06 硬件模型：dual_infantry（P0）

- 状态：待办
- 优先级：**P0**
- 影响机型：dual_infantry
- 依赖主题：05 硬件栈-librmcs与设备驱动
- 验收方式：`diff -u` 逐段核对两份 `dual_infantry.cpp`；核对两份 `dual-infantry.yaml`；`librmcs_core.so` 插件符号与 `.o` 时间戳核对；实机冒烟（两板卡枚举、CAN 收发、DR16、IMU、TF）

> 本文所有结论来自真实文件读取与 `diff -u`。被对比文件：
> - v2：`/workspaces/reATRM/src/core/utils/system_ctrl/rmcs_core/src/hardware/model/dual_infantry.cpp`（449 行）
> - v3：`/workspaces/RMCS/rmcs_ws/src/rmcs_core/src/hardware/dual_infantry.cpp`（427 行）
> - v2 config：`/workspaces/reATRM/src/bringup/config/dual-infantry.yaml`（234 行）
> - v3 config：`/workspaces/RMCS/rmcs_ws/src/rmcs_bringup/config/dual-infantry.yaml`（172 行）

## 1. v2（reATRM）现状

### 1.1 总体结构

- `class DualInfantry : public rmcs_executor::Component, public rclcpp::Node`（第 28–30 行）。
- 两个板对象：`TopBoard`（第 94 行，`private librmcs::client::CBoard`）、`BottomBoard`（第 296 行，`private librmcs::client::CBoard`）；由 `std::unique_ptr<TopBoard> top_board_`、`std::unique_ptr<BottomBoard> bottom_board_` 持有（第 441–442 行）。
- 通过 `create_partner_component<DualInfantryCommand>(...)` 生成命令组件（第 36–38 行）；`DualInfantryCommand::update()` → `DualInfantry::command_update()`（第 88 行）。
- 构造函数参数（第 50–55 行）：`usb_pid_top_board`、`usb_pid_bottom_board`（`as_int()`）。
- `update()` 只调用 `top_board_->update(); bottom_board_->update();`（第 60–63 行）。

### 1.2 板卡与总线

| 板 | 构造 | 事件线程 | CAN | UART |
|---|---|---|---|---|
| TopBoard | `librmcs::client::CBoard(usb_pid)`（第 100 行） | `std::thread event_thread_([this]{ handle_events(); })`（第 115 行），析构 `stop_handling_events()+join()`（第 161–164 行） | CAN1 + CAN2 | `dbus`（DR16，`dbus_receive_callback`）、UART2（GY614，`uart2_receive_callback`） |
| BottomBoard | `CBoard(usb_pid)`（第 302 行） | 同上（第 317、338–341 行） | CAN1（CAN2 空实现，第 397–404 行） | UART1（裁判系统，`uart1_receive_callback` → ring buffer；`referee_serial_->write` → `add_uart1_transmission`，第 324–332、406–409 行） |

### 1.3 电机与 CAN 配置（v2）

TopBoard 电机（第 109–113、117–140 行）：

| 实例 | 名称前缀 | 类型/配置 | 配置来源 |
|---|---|---|---|
| `gimbal_top_yaw_motor_` | `/gimbal/top_yaw` | `DjiMotor::Type::GM6020`，`set_encoder_zero_point(top_yaw_motor_zero_point)` | 无 id |
| `gimbal_pitch_motor_` | `/gimbal/pitch` | `DmMotor::Type::J4310`，`set_encoder_zero_point(pitch_motor_zero_point).set_reversed()` | — |
| `gimbal_left_friction_` | `/gimbal/left_friction` | `M3508`，`set_reduction_ratio(1.).set_reversed()` | — |
| `gimbal_right_friction_` | `/gimbal/right_friction` | `M3508`，`set_reduction_ratio(1.)` | — |
| `gimbal_bullet_feeder_` | `/gimbal/bullet_feeder` | `M3508`，`enable_multi_turn_angle().set_reversed().set_reduction_ratio(19*2)` | — |

TopBoard 收（`can1_receive_callback`，第 220–235 行）：`0x206`→top_yaw，`0x202`→left_friction，`0x201`→right_friction，`0x203`→bullet_feeder。
TopBoard 收（`can2_receive_callback`，第 237–246 行）：`0x213`→pitch（DM）。
TopBoard 发（`command_update`，第 196–217 行）：CAN1 `0x1FF`（[0]=0,[1]=top_yaw,[2]=0,[3]=0）；CAN1 `0x200`（[0]=right_friction,[1]=left_friction,[2]=bullet_feeder,[3]=0）；**CAN2 `0x3`** = `gimbal_pitch_motor_.generate_torque_command()`；`add_buzzer_transmission(buzzer_.generate_command())`。

BottomBoard 电机（第 305–314、319–322 行）：

| 实例 | 名称前缀 | 类型/配置 |
|---|---|---|
| `gimbal_bottom_yaw_motor_` | `/gimbal/bottom_yaw` | `GM6020`，`set_encoder_zero_point(bottom_yaw_motor_zero_point)` |
| `chassis_wheel_motors_[0]` | `/chassis/left_front_wheel` | `M3508` |
| `chassis_wheel_motors_[1]` | `/chassis/right_front_wheel` | `M3508` |
| `chassis_wheel_motors_[2]` | `/chassis/right_back_wheel` | `M3508` |
| `chassis_wheel_motors_[3]` | `/chassis/left_back_wheel` | `M3508` |
| `supercap_` | — | `device::Supercap(dual_infantry, 28.5)`（第 315 行） |

BottomBoard 收（第 376–395 行）：`0x205`→bottom_yaw，`0x20c`→supercap，`0x201..0x204`→4 个轮电机。
BottomBoard 发（第 359–373 行）：CAN1 `0x200`（4 轮）；CAN1 `0x1FF`（[0]=bottom_yaw）；无 buzzer。

### 1.4 IMU / 遥控 / 传感器

- IMU：TopBoard/BottomBoard 各持一个 `device::Bmi088 imu_(10.0f, 0.001f, 1000000.0f)`（第 102、303 行），即 librmcs EKF。加速度/角速度经 `store_accelerometer_status/store_gyroscope_status` 写入（第 256–262、411–417 行）。TopBoard 对陀螺仪减 `imu_bias_x/y/z`（第 261 行）；BottomBoard 不减（第 416 行）。
- TopBoard 设置 `imu_.set_coordinate_mapping(...)`，实际返回 `(x,y,z)` 恒等映射（第 142–153 行）。
- 遥控：TopBoard `device::Dr16 dr16_{dual_infantry}`（第 104 行），`dbus_receive_callback` → `dr16_.store_status(...)`。
- 蜂鸣器：TopBoard `device::Buzzer buzzer_(dual_infantry_command)`（第 105 行）。
- 摩擦轮温度：TopBoard `device::Gy614 gy614_(dual_infantry, "/friction_wheels/temperature")`（第 103 行），`uart2_receive_callback` → `gy614_.store_status`（第 252–254 行）。

### 1.5 TF

- 构造：`register_output("/tf", tf_)`；`tf_->set_transform<PitchLink, CameraLink>(Translation3d{0.084,0,0.048})`；`set_transform<PitchLink, MuzzleLink>(Translation3d{0,0,0})`（第 41–43 行）。
- TopBoard `update()`：`set_transform<PitchLink, OdomImu>(gimbal_imu_pose.conjugate())`；`set_state<YawLink, PitchLink>(pitch_motor.angle())`；**`fast_tf::rcl::broadcast_all(*tf_)`**；`set_transform<BaseLink, RawImu>(gimbal_imu_pose)`（第 173–180 行）。
- BottomBoard `update()`：`set_state<GimbalCenterLink, YawLink>(bottom_yaw_motor.angle())`；**`broadcast_all(*tf_)`**（第 349–351 行）。

### 1.6 注册接口清单（v2）

- `/tf`
- `/gimbal/yaw/velocity_imu`、`/gimbal/pitch/velocity_imu`
- `/debug/pitch/raw_angle`
- （声明但**未注册**）`debug_imu_g_x_/y_/z_`（第 274–276 行）——死成员
- `/referee/serial`
- `/chassis/yaw/velocity_imu`
- 间接注册（由设备构造）：`/gimbal/*` 与 `/chassis/*` 各电机的 `angle/raw_angle/velocity/torque/max_torque/velocity_filtered/alive` 与 inputs `…/control_torque`（DM 另有 `…/control_velocity`）；`/remote/*` 全套；`/buzzer/score`；`/friction_wheels/temperature`；`/chassis/power`、`/chassis/supercap/{voltage,energy_percentage,enabled}`。

## 2. v3（jRMCS）现状

### 2.1 总体结构

- `class DualInfantry : public rmcs_executor::Component, public rclcpp::Node`（第 34–36 行），与 v2 一致。
- 新增顶层成员 `std::unique_ptr<device::RemoteControl> remote_control_`（第 414 行，构造于第 55 行）。
- 两个板对象 `TopBoard`（第 102 行）/`BottomBoard`（第 283 行）均 `: public librmcs::board::CBoard::Callback`（**不再是 CBoard 私有继承**）。
- 构造参数（第 56–59 行）：`board_serial_top_board`、`board_serial_bottom_board`（`as_string()`）。
- `update()`（第 64–71 行）：`top_board_->update(); bottom_board_->update(); remote_control_->update();` 并按 `kTfBroadcastDivider = 10` 节流广播 `/tf`（100 Hz）。
- `template` 命令组件同 v2（第 91–100 行）。

### 2.2 板卡与总线

| 板 | 构造 | 事件线程 | CAN | UART |
|---|---|---|---|---|
| TopBoard | `std::make_unique<librmcs::board::CBoard>(*this, board_serial)`（第 154 行，构造体末尾） | SDK 内部；`~TopBoard() final = default`（第 157 行） | `Spec::kCans.kCan1`（**仅 CAN1**） | `Spec::kUarts.kDbus`（DR16，第 236–239 行）；**无 UART2/GY614** |
| BottomBoard | 同上（第 318 行） | 同上（第 321 行） | `Spec::kCans.kCan1` | **无 UART1/裁判串口** |

### 2.3 电机与 CAN 配置（v3）

| 实例 | 名称前缀 | v3 类型 + id |
|---|---|---|
| `gimbal_top_yaw_motor_` | `/gimbal/top_yaw` | `DjiMotor::Type::kGM6020Voltage, 2`（第 117 行） |
| `gimbal_pitch_motor_` | `/gimbal/pitch` | `DmMotor::Type::kJ4310`（第 122 行），`set_reversed()` **先于** `set_encoder_zero_point` |
| `gimbal_left_friction_` | `/gimbal/left_friction` | `kM3508, 2`，`set_reduction_ratio(1.).set_reversed()`（第 128 行） |
| `gimbal_right_friction_` | `/gimbal/right_friction` | `kM3508, 1`，`set_reduction_ratio(1.)`（第 132 行） |
| `gimbal_bullet_feeder_` | `/gimbal/bullet_feeder` | `kM3508, 3`，`enable_multi_turn_angle().set_reversed().set_reduction_ratio(19*2)`（第 136 行） |
| `gimbal_bottom_yaw_motor_` | `/gimbal/bottom_yaw` | `kGM6020Voltage, 1`（第 302 行） |
| `chassis_wheel_motors_[0..3]` | left_front / right_front / right_back / left_back | `kM3508, 1..4`（第 293–299 行） |

v3 收（TopBoard `can_receive_callback`，第 216–234 行）：只处理 `kCan1`；用 `recv_id()` 匹配 top_yaw(`0x206`)、left_friction(`0x202`)、right_friction(`0x201`)、bullet_feeder(`0x203`)，`0x213`→pitch。
v3 发（`command_update`，第 197–213 行）：`kCan1 0x1FF`（PaddingQuarter, top_yaw, Pad, Pad）；`kCan1 0x200`（right_friction, left_friction, bullet_feeder, Pad）；**`kCan1 0x3` = pitch**。
v3 BottomBoard 收（第 366–380 行）：只处理 `kCan1`；`recv_id()`/`match_then_store_status()` 匹配 bottom_yaw(`0x205`) 与 4 轮(`0x201..0x204`)。
v3 BottomBoard 发（第 348–363 行）：`kCan1 0x200`（4 轮）；`kCan1 0x1FF`（bottom_yaw + 3×Pad）。

### 2.4 IMU / 遥控 / 传感器（v3）

- IMU：TopBoard `device::Bmi088Ekf imu_`，`Config{Eigen::Matrix3d{{0,1,0},{1,0,0},{0,0,-1}}}`（第 255–259 行，即 `body_to_sensor`）；BottomBoard `device::Bmi088Ekf imu_{}`（默认单位阵，第 394 行）。
- 时间戳：两板各持 `device::BoardClockLifter board_clock_lifter_`（第 260、395 行）；加速度走 `advance_timebase`，角速度走 `lift_timestamp`（第 241–251、382–392 行）。
- 遥控：`remote_control_ = std::make_unique<device::RemoteControl>(*this)`（第 55 行），TopBoard 内 `remote_control_->register_dr16(&dr16_)`（第 152 行）；`Dr16` 仍由 `kDbus` 喂。
- **无蜂鸣器、无 GY614 温度、无 supercap、无裁判串口**。
- gyro 用途：TopBoard `gimbal_yaw_velocity_imu_ = filter(gyro_body.z())`、`gimbal_pitch_velocity_imu_ = filter(gyro_body.y())`（第 177–178 行）；滤波器 `imu_gy=10.0f`（v2 为 3.0f）、`imu_gz=30.0f`（同 v2）。
- 姿态：`tf_->set_transform<PitchLink, OdomImu>(pose.conjugate())`、`set_transform<BaseLink, RawImu>(pose)`（第 172–175 行），与 v2 一致，但姿态来自 `imu_.snapshot()->orientation`。

### 2.5 TF（v3）

- 构造同 v2：`set_transform<PitchLink, CameraLink>({0.084,0,0.048})`、`set_transform<PitchLink, MuzzleLink>({0,0,0})`（第 47–48 行）。
- `broadcast_all` **移出** TopBoard/BottomBoard，改为 `DualInfantry::update()` 内按 `kTfBroadcastDivider` 节流（第 69–70 行）。
- TopBoard：`set_state<YawLink, PitchLink>(pitch_motor.angle())`（第 185 行）。
- BottomBoard：`set_state<GimbalCenterLink, YawLink>(bottom_yaw_motor.angle())`（第 341 行）。

### 2.6 注册接口清单（v3）

- `/tf`
- `/gimbal/yaw/velocity_imu`、`/gimbal/pitch/velocity_imu`
- `/debug/pitch/raw_angle`
- `/debug/top_board/imu_gyro_{x,y,z}`、`/debug/top_board/imu_{roll,pitch,yaw}`
- `/chassis/yaw/velocity_imu`
- `/debug/bottom_board/imu_gyro_{x,y,z}`、`/debug/bottom_board/imu_{roll,pitch,yaw}`
- 间接注册：`/remote/*`（由 `RemoteControl`）、各电机 `…/angle,raw_angle,velocity,torque,max_torque,alive` + input `…/control_torque`（DM 另有 `…/control_velocity,temperature,error_code,velocity_filtered`）。
- **不再注册**：`/referee/serial`、`/friction_wheels/temperature`、`/buzzer/score`、`/chassis/power`、`/chassis/supercap/*`。

## 3. 差异清单

### 3.1 硬件模型（`dual_infantry.cpp`）

| # | 差异点 | v2 做法 | v3 做法 | 影响 | 风险 |
|---|---|---|---|---|---|
| M1 | 板卡 API | `private librmcs::client::CBoard(usb_pid)` + 手写事件线程 | `public librmcs::board::CBoard::Callback` + `librmcs::board::CBoard(*this, board_serial)`，线程由 SDK 管理 | 构造/析构与回调签名整体改写 | 中 |
| M2 | 板卡识别 | `usb_pid_top_board: 0x9d15`、`usb_pid_bottom_board: 0x34a4` | `board_serial_top_board: "D4-2EEA-E236-F8E4-62C8-9974-F3A5"`、`board_serial_bottom_board: "D4-7C74-3B6F-7B82-B341-E573-0470"`；SDK PID 固定 0xD401 | 板卡选择语义变化 | **高**：serial 不符则构造抛异常，机型完全起不来 |
| M3 | 电机 id 必须显式 | v2 `Config{Type}` 无 id，id 隐含在 `can*_receive_callback` 的 `can_id` 分支里 | v3 `Config{Type, id}`；id 参与 `recv_id()/send_id()`，并用于组包 `operator<<` | 需为每台电机确定物理 CAN id | **高**：id 填错 → 收不到反馈 / 发错槽位 |
| M4 | 收发集中在 CAN1 | TopBoard pitch 在 **CAN2**（收 `0x213`、发 `0x3`），其余在 CAN1 | TopBoard pitch 改到 **kCan1**（收 `0x213`、发 `0x3`） | 若实机 pitch 仍接 CAN2，则 v3 收不到 DM 反馈、也发不出指令 | **高**：需与实机接线核对（**未核实**是接线已改还是迁移笔误） |
| M5 | yaw 电机类型 | `DjiMotor::Type::GM6020`（电流环语义，`send_id=0x1FE`），但命令实际发到 `0x1FF` | `DjiMotor::Type::kGM6020Voltage`（电压环，`send_id=0x1FF`），与发送 id 一致 | `GM6020Voltage` 的 `raw_current_max=25000`（v2 librmcs `GM6020` 为 `16384`）→ **转矩换算系数不同** | **高**：同一 `control_torque` 会给出不同电流指令（约 ×0.655），若不重标定会明显变"软/过冲" |
| M6 | `GM6020Voltage` 命名与 id | v2 `Config{Type::GM6020}`（无 id） | v3 `Config{kGM6020Voltage, 2}`（top_yaw）、`{kGM6020Voltage, 1}`（bottom_yaw） | 与实机 `0x206`/`0x205` 反馈 id 一致（已核对 `recv_id()=0x204+id`） | 中（正确性取决于实机 id） |
| M7 | `use of kCan2` | TopBoard 有 `can2_receive_callback` 处理 pitch | TopBoard 无 CAN2 分支 | 见 M4 | 高（同 M4） |
| M8 | IMU 驱动 | `device::Bmi088`（librmcs EKF，构造 `(10,0.001,1e6)`）+ `set_coordinate_mapping(恒等)` | `device::Bmi088Ekf` + `BoardClockLifter`；TopBoard `Config{body_to_sensor={{0,1,0},{1,0,0},{0,0,-1}}}`，BottomBoard 默认单位阵 | 姿态/角速度来源改为 `snapshot()`；TopBoard 坐标映射由"恒等"变为矩阵 | **高**：TopBoard 映射由恒等变为非平凡置换，若与实机不符则 roll/pitch/yaw 全错 |
| M9 | IMU 偏置 | `imu_bias_x/y/z` 参数，作用于 TopBoard 角速度，配置为 `3/8/2` | v3 dual_infantry **无** imu_bias（参数已从 config 删除，代码也不读） | 零偏补偿丢失 | 中：可能造成 yaw 漂移，需用 EKF/标定补回 |
| M10 | 角速度滤波 | `imu_gy` 用 3.0 Hz | `imu_gy` 用 10.0 Hz（`imu_gz` 两版均 30 Hz） | pitch 速度环相位/噪声特性变化 | 中：需重调 pitch 速度环 |
| M11 | 遥控注册 | `Dr16` 与 `Buzzer` 分别在 TopBoard 内自我注册 `/remote/*` | `RemoteControl` 统一注册；`register_dr16(&dr16_)`（`vt13` 未注册） | `/remote/*` 输出方变化；因只注册 dr16，`dr16_->set_timeout_enabled(false)` → DR16 永不超时 | 中：DR16 掉线不再自动归零（安全语义变化） |
| M12 | 蜂鸣器 | `device::Buzzer` + `add_buzzer_transmission` | 完全移除 | 失去蜂鸣提示 | 中（功能缺口；主题 05 A11） |
| M13 | 摩擦轮温度 | `device::Gy614`（UART2）→ `/friction_wheels/temperature` | 完全移除（dual_sentry 仍保留 GY614） | 失去摩擦轮温度监测 | 中（功能缺口） |
| M14 | 超级电容 | `device::Supercap(..., 28.5)`；输出 `/chassis/power`、`/chassis/supercap/{voltage,energy_percentage,enabled}` | 完全移除 | 失去底盘功率/超电状态 | **高**（功能缺口：底盘功率控制链路断） |
| M15 | 裁判串口 | BottomBoard 注册 `/referee/serial`（UART1 双向） | 完全移除；dual_sentry/hero/mini 都保留 | `rmcs_core::referee::Command` 的输入 `/referee/serial` 无来源 | **高**：裁判系统整链失效 |
| M16 | TF 广播 | TopBoard/BottomBoard 内各自 `broadcast_all`（每板每周期一次） | 顶层 `update()` 内按 10 分频广播（100 Hz） | 广播频率显式降为 100 Hz | 中：依赖 >100 Hz TF 的消费方需评估 |
| M17 | debug 输出 | `/debug/pitch/raw_angle`；`debug_imu_g_x/_y/_z` 仅声明未注册 | 新增 `/debug/top_board/imu_gyro_{x,y,z}`、`/debug/top_board/imu_{roll,pitch,yaw}`、`/debug/bottom_board/*` 同名 6 项 | 调参可视化更完整 | 低 |
| M18 | 姿态 debug 计算 | 无（v2 未算欧拉角） | v3 用旋转矩阵反算 roll/pitch/yaw（第 167–170、334–337 行） | 新增能力 | 低 |
| M19 | `DmMotor` 配置顺序 | `set_encoder_zero_point(...).set_reversed()` | `set_reversed().set_encoder_zero_point(...)` | 语义等价（链式调用顺序不影响最终配置值） | 低 |
| M20 | 电机 `velocity_filtered` | TopBoard 全部电机（DJI 亦有） | `DjiMotor` 无 `velocity_filtered`，`DmMotor` 有 | PID/broadcaster 引用需改 | 中 |
| M21 | 余量/健壮性 | `can_data_length < 8` 时丢弃 | `store_status` 内 `can_data.size() != 8` 丢弃；`is_extended_can_id/is_remote_transmission` 丢弃 | 等价 | 低 |

### 3.2 Config 差异（`dual-infantry.yaml`）

**组件清单**

| 组件 | v2 | v3 | 说明 |
|---|---|---|---|
| `hardware::DualInfantry -> dual_infantry_hardware` | ✅ | ✅ | 同名 |
| `referee::Status -> referee_status` | ✅ | ❌ | 缺失 |
| `referee::Command -> referee_command` | ✅ | ❌ | 缺失 |
| `referee::command::Interaction -> referee_interaction` | ✅ | ❌ | 缺失 |
| `referee::command::interaction::Ui -> referee_ui` | ✅ | ❌ | 缺失 |
| `referee::app::ui::DualInfantry -> dual_infantry_ui` | ✅ | ❌ | v3 已无该类（v3 用 `referee::app::ui::Infantry`，本机型 config 也未挂） |
| `SimpleGimbalController -> gimbal_controller` | ✅ | ✅ | — |
| `DualYawSolver -> dual_yaw_slover` | ✅（拼写 `slover`） | ✅（`dual_yaw_solver`） | 实例名拼写修正，**注意参数块键名随之改变** |
| `DualYawController -> dual_yaw_controller` | ✅ | ❌ | v3 换成 `SmcDualYawController -> dual_yaw_controller` |
| `ErrorPidController -> pitch_angle_err_pid_controller` | ✅ | ✅ | — |
| `PidController -> pitch_velocity_pid_controller` | ✅ | ✅ | — |
| `chassis::OperateLogic -> operate_logic` | ✅ | ❌ | 缺失 |
| `shooting::FrictionWheelController -> friction_wheel_controller` | ✅ | ✅ | — |
| `shooting::HeatController -> heat_controller` | ✅ | ❌ | 缺失（且 v3 config 无 `heat_controller` 参数块） |
| `shooting::BulletFeederController17mm -> bullet_feeder_controller` | ✅ | ✅ | — |
| `PidController -> {left,right}_friction_velocity_pid_controller` | ✅ | ✅ | — |
| `PidController -> bullet_feeder_velocity_pid_controller` | ✅ | ✅ | — |
| `rmcs::AutoAimComponent -> auto_aim_component` | ✅ | ❌ | 缺失（需确认是否改由 `rmcs_auto_aim_v2` 其他方式接入） |
| `chassis::ChassisController -> chassis_controller` | ✅ | ✅ | — |
| `chassis::ChassisPowerController -> chassis_power_controller` | ✅ | ❌ | v3 已有 `JChassisPowerController`，本机型未挂 |
| `chassis::OmniWheelController -> omni_wheel_controller` | ✅ | ❌ | v3 换成 `JOmniWheelController -> omni_wheel_controller` |
| `diagnosis::Diagnosis -> diagnosis` | ✅ | ❌ | 缺失（v3 其它机型仍在用） |
| `broadcaster::ValueBroadcaster -> value_broadcaster` | ✅ | ✅ | 名单不同 |

**`dual_infantry_hardware` 参数块**

| 参数 | v2 | v3 |
|---|---|---|
| 板卡标识 | `usb_pid_top_board: 0x9d15` / `usb_pid_bottom_board: 0x34a4` | `board_serial_top_board: "D4-…F3A5"` / `board_serial_bottom_board: "D4-…0470"` |
| `bottom_yaw_motor_zero_point` | 6705 | 2687 |
| `top_yaw_motor_zero_point` | 7850 | 7916 |
| `pitch_motor_zero_point` | 35030 | 29364 |
| `imu_bias_x/y/z` | 3 / 8 / 2 | **已删除** |

> 零点三项差异可能来自「不同机器人实例」而非迁移本身（v3 三机型的 `board_serial` 多为 `placeholder-*`，仅 dual-infantry 填了真实值）；**未核实**是否为同一台车的重新标定值。

**其它参数块差异（同键不同值 / 键改名 / 新增）**

| 参数块 | v2 | v3 |
|---|---|---|
| `gimbal_controller` | `upper_limit: -0.28`、`lower_limit: 0.35` | `upper_limit: -0.36`、`lower_limit: 0.66`；新增 `yaw_upper_limit: 1e6`、`yaw_lower_limit: -1e6` |
| `dual_yaw_controller` | `DualYawController`：`top_yaw_angle_kp 15`、`top_yaw_velocity_kp 2.0/ki 0.01`、`bottom_yaw_smc_c 10/k 1.5/eps 1.0/phi 2.0/inertia 2.0`、`bottom_yaw_max_torque 16384.0` | `SmcDualYawController`：`top_yaw_angle_kp 18`、新增 `top_yaw_angle_output_{min,max}`/`top_yaw_velocity_output_{min,max}`、`bottom_yaw_smc_c 3/k 1.5/eps 0.3/phi 0.1/inertia 0.5`、`bottom_yaw_max_torque 2.0`、新增 `bottom_yaw_accel_ff_alpha 0.3`、`top_yaw_angle_limit 0.9` |
| `dual_yaw_solver`（v2 键名 `dual_yaw_slover`） | 无 | `top_yaw_limit: 0.7854`、`spin_angle_bias: 0.0` |
| `chassis_controller` | `translational_velocity_limit 99.0`、`angular_velocity_limit 25.0`、`autopilot_spin_velocity_default 2.0` | **键改名**：`translational_velocity_max 80.0`、`angular_velocity_max 15.0`（无 `autopilot_spin_velocity_default`） |
| `omni_wheel_controller` | `chassis_radius_x/y 0.165`、`wheel_radius_ 0.07`（键名带尾下划线）、`friction_coefficient 0.6`、`spinning_bais_coefficient -0.08` | `mass 15.0`、`moment_of_inertia 4.08`、`chassis_radius_x/y 0.165`、`wheel_radius 0.07`、`spinning_bias_adjust false`、`spinning_bias_coefficient -0.08`、`power_limit_default 45.0` |
| `pitch_angle_err_pid_controller` | `enable: /gimbal/pitch/alive`、`kp 8.0/ki 0.001/kd 1.0`、积分 ±5 | 无 `enable`、`kp 6.0/ki 0.0/kd 60.0`、积分 ±5 |
| `pitch_velocity_pid_controller` | `enable: /gimbal/pitch/alive`、`kp 2.5/ki 0.05/kd 45.0`、积分 ±25 | 无 `enable`、`kp 1.2/ki 0.0/kd 15.0`、积分 ±2 |
| `bullet_feeder_velocity_pid_controller` | `measurement: /gimbal/bullet_feeder/velocity_filtered`、`kp 5.0/kd 50.0` | `measurement: /gimbal/bullet_feeder/velocity`、`kp 0.0/kd 0.0` |
| `bullet_feeder_controller` | 无 `bullet_allowance_default` | 新增 `bullet_allowance_default: 100` |
| `value_broadcaster` | `forward_list` 13 项（含 `/gimbal/pitch/velocity_filtered`、`/chassis/angular_vmax`、`/debug/sliding_value` 等），无 `publish_divider` | `publish_divider: 10`，`forward_list` 仅 1 项 `/gimbal/yaw/control_angle_error` |
| `pid_tune_tool` | — | v3 无该机型文件（见主题 07） |

### 3.3 已迁移 / 未迁移（v3 dual_infantry）

- **已迁移**：板卡→SDK 新 API；全部电机（含显式 id）；IMU→`Bmi088Ekf` + `BoardClockLifter`；遥控→`RemoteControl`（仅挂 DR16）；TF（含节流广播）；`_command` 组件与 gimbal 校准订阅；debug 遥测。
- **未迁移（v3 缺失）**：裁判串口 `/referee/serial`（M15）、超级电容（M14）、蜂鸣器（M12）、GY614 摩擦轮温度（M13）、`imu_bias_*`（M9）、config 中的 referee/UI/heat/operate_logic/diagnosis/auto_aim/chassis_power 组件（见 3.2）。
- **行为改变**：pitch 由 CAN2 改到 CAN1（M4）；yaw 电机 `GM6020`→`kGM6020Voltage`（M5，换算系数变）；IMU 坐标映射由恒等改为矩阵（M8）；TF 100 Hz（M16）。
- **存疑**：M2 的 serial、M4 的 CAN 通道、M8 的坐标映射、三个零点参数是否为同一台车（均「未核实」，需实机/装机记录确认）。

### 3.4 编译状态推测

- **v3 现状：可编译、插件可注册（已核实）**：
  - 目标文件存在且**晚于源文件**：`dual_infantry.cpp` 修改于 2026-10-17 21:04，`build-cross-arm64/rmcs_core/CMakeFiles/rmcs_core.dir/src/hardware/dual_infantry.cpp.o` 生成于 21:07。
  - 最近一次包含 `rmcs_core` 的构建 `log-cross-arm64/build_2026-10-17_21-06-51/rmcs_core/stdout_stderr.log` 中 **`grep -iE "error|warning"` 无命中**，且完成了 install 符号链接步骤。
  - `nm -C install-cross-arm64/lib/librmcs_core.so` 中出现 `class_loader::impl::registerPlugin<rmcs_core::hardware::DualInfantry, rmcs_executor::Component>`。
  - 注意：`install-cross-arm64/lib/librmcs_core.so` 的时间戳为 2026-10-15 20:11，**早于** 10-17 的 `.o`；即「编译已通过、但已安装产物可能未重新链接/安装」，做最终验证时应重新构建。
- **结论**：缺失的 referee/supercap/buzzer/gy614 是**功能缺口而非编译错误**；`usb_pid_*`→`board_serial_*` 已同步到代码与 config，故不存在悬空参数。
- **运行时风险**（编译无法覆盖）：M2（serial 选择）、M4（CAN 通道）、M5（转矩系数）、M8（坐标映射）、M15（`referee::Command` 输入无来源）。

## 4. 迁移方案（以 v3 为骨架）

按「先能跑、再补齐、后校正」三步：

**阶段 A：可运行（保持 v3 结构，只补必要项）**
1. 获取并核对两板真实 serial，写入 `board_serial_top_board/bottom_board`；确认 SDK PID 0xD401 与实机一致。
2. **确认 pitch 的物理 CAN 通道**：若实机为 CAN2，则把 v3 的 `kCan1` 改回 `Spec::kCans.kCan2`（代码 3 处：发送 `can_id 0x3`、`can_receive_callback` 的通道过滤、pitch 的接收分支）。
3. **确认 yaw 电机类型**：若实机用电流环（0x1FE），把 `kGM6020Voltage` 改回 `kGM6020`；若实机用电压环（0x1FF），保留 v3 现状并**重标定** `control_torque→max_torque`（因 `raw_current_max` 25000 vs 16384）。
4. **确认 TopBoard IMU 坐标映射**：以 v2 的恒等映射为基线，逐个轴核对实机符号；`body_to_sensor` 需按 hero 的写法注释来源（v3 hero 有先例：`(0,1,0,-1,0,0,0,0,1)` 等）。
5. 恢复 `imu_bias_x/y/z`（若实机确有零偏），或在 `Bmi088Ekf` 标定流程中覆盖。

**阶段 B：补齐功能（对齐 v2 能力）**
6. 补 `referee_serial_`：仿 v3 `dual_sentry.cpp` 的 BottomBoard 写法（`register_output("/referee/serial", referee_serial_)` + `read` 用 `RingBuffer::pop_front_n` + `write` 用 `board_->start_transmit().uart_transmit(Spec::kUarts.kUart1, {...})`），并在 config 重新挂 `referee::Status/Command/Interaction/Ui`。
7. 补 supercap：二选一（`Jsupercap` 沿用老协议；或 `Supercap` 新协议 + 功率闭环组件）。若选 `Jsupercap`，在 BottomBoard 收 `0x20c` 并接 `chassis_power_controller`。
8. 补蜂鸣器：依赖主题 05 的结论（新驱动 + 下行支持，或确认取消）。
9. 补 GY614 温度（若该车有摩擦轮温度传感器）：TopBoard `uart_receive_callback` 加 `kUart2` 分支。
10. config 补齐：`heat_controller`、`operate_logic`、`diagnosis`、`auto_aim_component`、`chassis_power_controller`（按需），并修正 `bullet_feeder_velocity_pid_controller` 的 `measurement` 到 `/gimbal/bullet_feeder/velocity`（v3 已改，注意 v2 的 `velocity_filtered` 在 `DjiMotor` 已不存在）。

**阶段 C：参数与环境校正**
11. 重标零点和 PID：`dual_yaw_controller` 的控制器类已换（`DualYawController`→`SmcDualYawController`），`bottom_yaw_max_torque` 从 16384 变 2.0 语义不同，必须重调。
12. `chassis_controller`/`omni_wheel_controller` 键名已改（`_limit`→`_max`、`wheel_radius_`→`wheel_radius`）——确认无遗留旧键。
13. 恢复有价值的 `value_broadcaster.forward_list` 调参项（如 `/gimbal/pitch/velocity_imu`、`/chassis/power`、`/chassis/supercap/voltage`）。

## 5. 执行步骤 checklist

- [ ] 核对两板 serial 并写入 config；确认 SDK PID 0xD401
- [ ] 核对 pitch 的物理 CAN 通道（CAN1 vs CAN2），修正代码与接线
- [ ] 核对 yaw 电机类型（kGM6020 vs kGM6020Voltage）并重标定转矩
- [ ] 核对 TopBoard `body_to_sensor` 映射（含注释依据）
- [ ] 决定并实现 `imu_bias_*` 处理
- [ ] 补 `/referee/serial`（BottomBoard，UART1）
- [ ] 决定并接回 supercap（`Jsupercap` 或 `Supercap`）+ `chassis_power_controller`
- [ ] 处理蜂鸣器（依赖主题 05）
- [ ] 处理 GY614 摩擦轮温度（若实机需要）
- [ ] config 组件补齐与键名修正（含 `bullet_feeder_velocity_pid_controller` 的 `measurement`）
- [ ] 重调 `dual_yaw_controller`/`gimbal_controller`/PID 参数
- [ ] 恢复 `value_broadcaster.forward_list`
- [ ] 重新构建并安装（不要复用 2026-10-15 的 `librmcs_core.so`）
- [ ] 实机冒烟：两板枚举、CAN 收发在线、DR16 遥控、IMU 姿态/TF、裁判数据

## 6. 验收与验证

- **静态**：`diff -u` 复核本文件「3.1/3.2」每一条；确认所有 config 中引用的接口都有硬件模型 register 来源。
- **编译**：`./.script/build-rmcs-cross --target-arch=arm64 --packages-skip rmcs_auto_aim_v2 --executor sequential` 通过且 `rmcs_core` 无 warning。
- **插件**：`nm -C install-cross-arm64/lib/librmcs_core.so | grep 'registerPlugin<rmcs_core::hardware::DualInfantry'` 命中。
- **接口连线**：`dual_infantry_hardware` 的 `/tf`、`/gimbal/{yaw,pitch}/velocity_imu`、`/chassis/yaw/velocity_imu`、`/debug/*` 与 `remote_control` 的 `/remote/*` 均可在 runtime 接口表中找到；`referee::Command` 的 `/referee/serial`（补回后）就绪。
- **冒烟（需硬件）**：
  - 两块板卡按 serial 正确区分（拔掉一块应匹配到正确的那块）。
  - CAN：top_yaw 收 `0x206` 转动、pitch 收 `0x213`、4 轮收 `0x201..0x204`；发送帧 id 正确。
  - 遥控：DR16 挡位/摇杆/鼠标/键盘/旋钮输出正常；拔掉 DBUS 的行为符合预期（当前不会自动归零，见 M11）。
  - IMU：`/debug/top_board/imu_{roll,pitch,yaw}` 与实机姿态一致；`/tf` 100 Hz。
  - 校准：调用 `/gimbal/calibrate` 后三个零点日志正确。
- **回归**：其它机型（dual-sentry/hero/mini-infantry）不受本机型改动影响。

## 7. 备注与未决问题

1. **M4（pitch 通道）最需要先定**：v3 把 DM pitch 放在 `kCan1`，而 v3 的 dual_sentry/hero 仍放在 `kCan2`——这是本机型独有的差异，可能是迁移笔误或接线变更，**未核实**。
2. **M5（转矩换算）**：`GM6020` 与 `GM6020Voltage` 的 `raw_current_max` 不同，若实机从未改过电机控制模式，v3 会改变实际电流输出；必须与实机确认。
3. **M8（坐标映射）**：v2 的恒等映射被替换为置换矩阵；v3 hero 的注释写法可作为参考（`hero.cpp:219-220`、`hero.cpp:361-362`），但 dual_infantry 的具体映射仍需实测校验。
4. **零点参数**：v2/v3 三组零点全不同，可能是「不同车」——**未核实**；若为同一台车，说明标定基准也变了。
5. **M11（DR16 超时）**：v3 只注册 DR16，`RemoteControl::update_timeout_interlock()` 使 `dr16_->set_timeout_enabled(false)`，即 DR16 掉线不再自动进入安全态。是否需要恢复超时（引入 VT13 或改互锁策略）待定。
6. **M16（TF 100 Hz）**：`kTfBroadcastDivider=10` 是显式注释的性能取舍；若自瞄/上位机需要更高频率需回调。
7. **config 中 processor 组件缺失**：`Referee`/`HeatController`/`OperateLogic`/`Diagnosis`/`AutoAimComponent` 在 v3 本机型 config 中均缺失，需与「迁移总任务」的控制器/裁判主题（其它文档）协调，本主题只记录它们与硬件模型的接口依赖。
8. **未核实项汇总**：板卡真实 serial、pitch 物理总线、yaw 电机控制模式、IMU 映射、零点来源、`imu_bias` 是否仍有必要。

---

## 迁移状态（阶段二，2026-10-19）

**已完成。** 改动文件：`rmcs_ws/src/rmcs_core/src/hardware/dual_infantry.cpp`。
- 接回 `/referee/serial`（BottomBoard UART1）、`Jsupercap`(28.5)、`Gy614` 摩擦轮温度（TopBoard UART2）、`imu_bias_x/y/z`。
- pitch 回到 **CAN2**（收 0x213 / 发 0x3）；yaw 恢复 `kGM6020` 电流环；TopBoard IMU 恢复恒等映射（均带 TODO 待实机标定）。
- 蜂鸣器因 v3 SDK 无 API 未移植（见主题 12）。
- 另为主题 17 新增 `/gimbal/auto_aim/imu_snapshot` 与可选 `/gimbal/auto_aim/exposure_signal` 事件输出。
- 验证：arm64 交叉编译 7 包全绿、无 warning。
