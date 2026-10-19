# bringup 配置与 launch

- 状态：待办
- 优先级：P0
- 影响机型：dual-infantry（P0）、dual-sentry、mini-infantry、mec-hero、auto_aim_test、navigation_test、pid-tune-tool（v2 专有）
- 依赖主题：`01-仓库布局与包结构`、`06-硬件模型-dual_infantry`、`08-控制器-底盘`、`09-控制器-云台`、`10-控制器-发射`、`12-诊断-diagnosis`、`13-裁判系统-Status与Command`、`14-裁判系统-UI`、`17-视觉子系统-rmcs_auto_aim_v2`
- 验收方式：
  1. `./.script/build-rmcs-cross --target-arch=arm64` 通过；
  2. `set-robot <name> && launch-rmcs` 后，`rmcs_executor` 内目标机型的全部组件均成功加载（无 `ParameterNotDeclared` / plugin 未找到），节点不重启（`respawn=True` 反复重启即为失败信号）；
  3. `foxglove`/`rmcs-cli` 观察到关键 topic 有数据（尤其 `/auto_aim/*`、`/gimbal/*`、`/chassis/*`）；
  4. `set-robot dual-infantry` 全链路联调（云台接管 + 发弹 + UI 显示）。

---

## 1. v2（reATRM）现状

### 1.1 包与文件

- 路径：`/workspaces/reATRM/src/bringup/`，包名 **`atrm_bringup`**（`CMakeLists.txt:2 project(atrm_bringup)`、`package.xml:5 <name>atrm_bringup</name>`）
- `package.xml` 依赖：`rm_serial_driver`、`pb2025_robot_description`、`pb2025_nav_bringup`、`rm_vision_bringup`；`exec_depend`：`joint_state_broadcaster / robot_state_publisher / rviz2 / xacro`
- `CMakeLists.txt`：`find_package(ament_cmake_auto)` + `ament_auto_package(INSTALL_TO_SHARE launch config)`（**一次性**安装）
- 文件清单（`find`）：
  ```
  CMakeLists.txt  package.xml
  config/{dual-infantry,dual-sentry,mec-hero,mini-infantry,pid-tune-tool}.yaml   # 5 份
  launch/rmcs.launch.py
  ```

### 1.2 launch（`launch/rmcs.launch.py`）

```python
36:        enable_vision_entities = ["dual-sentry", "dual-infantry", "mec-hero", "mini-infantry"]
...
38:        entities.append(Node(
39:            package="rmcs_executor", executable="rmcs_executor",
40:            parameters=[os.path.join(
44:                FindPackageShare("atrm_bringup").perform(context), "config", robot_name + ".yaml")],
47:            respawn=True, respawn_delay=1.0, output="log"))
...
56:        if robot_name in enable_vision_entities:
57:            entities.append(Node(package="rmcs_auto_aim_v2",
60:                executable="rmcs_auto_aim_v2_runtime",
61:                respawn=True, respawn_delay=1.0, output="screen"))
```

- 即：**1 个 `rmcs_executor` 节点 + （4 个机型额外）1 个独立自瞄进程**。
- 入口脚本：`/workspaces/reATRM/.script/start` → `ros2 launch atrm_bringup rmcs.launch.py robot:=${RMCS_ROBOT_TYPE}`（要求 `RMCS_ROBOT_TYPE` 非空）。

### 1.3 各 config 的组件清单（`components:` 段原样）

**dual-infantry.yaml**（23 个组件，`:5-35`）

```
hardware::DualInfantry -> dual_infantry_hardware
referee::Status -> referee_status / referee::Command -> referee_command
referee::command::Interaction -> referee_interaction
referee::command::interaction::Ui -> referee_ui
referee::app::ui::DualInfantry -> dual_infantry_ui
gimbal::SimpleGimbalController -> gimbal_controller
gimbal::DualYawSolver -> dual_yaw_slover          # 注意：v2 实例名拼写 slover
gimbal::DualYawController -> dual_yaw_controller
pid::ErrorPidController -> pitch_angle_err_pid_controller
pid::PidController -> pitch_velocity_pid_controller
chassis::OperateLogic -> operate_logic            # ★ v2 独有
shooting::FrictionWheelController -> friction_wheel_controller
shooting::HeatController -> heat_controller
shooting::BulletFeederController17mm -> bullet_feeder_controller
pid::PidController -> {left,right}_friction_velocity_pid_controller
pid::PidController -> bullet_feeder_velocity_pid_controller
rmcs::AutoAimComponent -> auto_aim_component
chassis::ChassisController -> chassis_controller
chassis::ChassisPowerController -> chassis_power_controller
chassis::OmniWheelController -> omni_wheel_controller   # ★ v2 独有
diagnosis::Diagnosis -> diagnosis
broadcaster::ValueBroadcaster -> value_broadcaster
```

**dual-sentry.yaml**（24 个）：同上另加 `referee::app::ui::DualSentry -> dual_sentry`、`autopilot::AutoPilotComponent -> auto_pilot_component`；底盘为 `ChassisPowerController` + `OmniWheelController`。

**mec-hero.yaml**：`hardware::Hero`、`ui::Hero`、`ChassisPowerController` + `OmniWheelController`、42mm 弹链 + 三个摩擦轮 PID（与 v3 结构一致，类名不同）。

**mini-infantry.yaml**：`hardware::MiniInfantry`、`ui::MiniInfantry`、17mm 弹链、`ChassisPowerController` + `OmniWheelController`。

**pid-tune-tool.yaml**（v2 独有，`:1-48`）：`hardware::PidTuneTool -> hardware`、`broadcaster::ValueBroadcaster`、`pid::PidController -> velocity_pid_controller`；参数块 `velocity_pid_controller`（`measurement: /top/imu/gz`、`kp 3.0/ki 0.02/kd 40.0`）、`hardware`（`usb_pid_top_board: 0xa513`、`usb_pid_bottom_board: 0x4ca4`、`motor_zero_point`、`velocity_step: 1.0`、`step_time: 400.0`、`imu_bias_*`）、`value_broadcaster.forward_list`（13 项）。

---

## 2. v3（jRMCS）现状

### 2.1 包与文件

- 路径：`/workspaces/RMCS/rmcs_ws/src/rmcs_bringup/`，包名 **`rmcs_bringup`**
- `package.xml`：**只保留** `exec_depend {joint_state_broadcaster, robot_state_publisher, rviz2, xacro}` + `test_depend`；**没有** v2 的 `rm_serial_driver / pb2025_* / rm_vision_bringup`，也**没有**显式 `exec_depend rmcs_core / rmcs_auto_aim_v2`（依赖靠 `rmcs_executor` pluginlib 动态加载，**未核实**是否影响构建顺序）
- `CMakeLists.txt`：`find_package(ament_cmake)`；`install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})` **重复写了两次**（`:16-19` 与 `:21-24`，无害冗余）
- 文件清单：
  ```
  CMakeLists.txt  LICENSE  package.xml  launch/rmcs.launch.py
  config/{auto_aim_test,dual-infantry,dual-sentry,mec-hero,mini-infantry,navigation_test}.yaml   # 6 份
  config/legacy/{deformable-infantry-omni-b,deformable-infantry-omni,flight,omni-infantry,sentry,
                 steering-hero-little-six-friction}.yaml_                                        # 6 份停用
  ```

### 2.2 launch（`launch/rmcs.launch.py`）

- 与 v2 **同源**（同一个 `MyLaunchDescriptionEntity` 类），差异只有两处：`FindPackageShare("rmcs_bringup")`（v2 为 `atrm_bringup`）、**删除**了 `enable_vision_entities` 与自瞄节点块。
- 结果：`ros2 launch rmcs_bringup rmcs.launch.py robot:=<name>` 只起 **1 个 `rmcs_executor` 节点**，参数文件 `share/rmcs_bringup/config/<name>.yaml`，`respawn=True, respawn_delay=1.0, output="log"`。
- 入口脚本：`/workspaces/RMCS/.script/launch-rmcs` → `ros2 launch rmcs_bringup rmcs.launch.py robot:=${RMCS_ROBOT_TYPE}`；`RMCS_ROBOT_TYPE` 由 `set-robot` 写入 `~/env_setup.{bash,zsh}`。
- 与 `AGENTS.md` 第 4 节约定一致（「新增机器人只需新增同名 config/<robot>.yaml，无需改 launch」）。

### 2.3 各 config 的组件清单

**dual-infantry.yaml**（`components:` 只有 13 条，`:4-22`）

```
hardware::DualInfantry -> dual_infantry_hardware
gimbal::SimpleGimbalController -> gimbal_controller
gimbal::DualYawSolver -> dual_yaw_solver
gimbal::SmcDualYawController -> dual_yaw_controller      # ★ v3 独有
pid::ErrorPidController -> pitch_angle_err_pid_controller
pid::PidController -> pitch_velocity_pid_controller
shooting::FrictionWheelController -> friction_wheel_controller
shooting::BulletFeederController17mm -> bullet_feeder_controller
pid::PidController -> {left,right}_friction_velocity_pid_controller
pid::PidController -> bullet_feeder_velocity_pid_controller
chassis::ChassisController -> chassis_controller
chassis::JOmniWheelController -> omni_wheel_controller    # ★ v3 独有
broadcaster::ValueBroadcaster -> value_broadcaster
```

> **缺失**：`referee::{Status,Command}`、`referee::command::Interaction`、`referee::command::interaction::Ui`、UI 类、`shooting::HeatController`、`rmcs::AutoAimComponent`、`chassis::ChassisPowerController`、`diagnosis::Diagnosis`。这是**与其它 v3 机型配置不一致**的异常精简版（见 §3.1/§3.2）。

**dual-sentry / mec-hero / mini-infantry**：结构与 v2 基本一一对应（含 referee 全链 + UI + HeatController + Diagnosis + `rmcs::AutoAimComponent` + `JChassisPowerController` + `JOmniWheelController`），差异集中在**类名替换**与少量参数键名（§3.1、§3.7）。

**auto_aim_test.yaml**（v3 独有，`:1-52`）：只有 `rmcs::AutoAimVideoPlayerComponent`（注释掉 Player/Recorder）+ `rmcs::AutoAimComponent`；`auto_aim_component.fire_control` 给出完整参数（`bullet_speed 22.5 / shoot_delay 0.0 / attack_window 60 / require_stable_command: true …`，其中 `require_stable_command` 在 v3 源码中**无对应键**，见主题 17 §3.6）。

**navigation_test.yaml**（v3 独有，`:1-11`）：`rmcs::navigation::Navigation -> rmcs_navigation`（`command_vel_name: /cmd_vel`、`endpoint: "rmuc"`）；**该插件类在当前 `rmcs_ws/src` 中不存在对应包**（grep 全工作区仅命中此 yaml），属外部/未 vendored 包，**未核实**是否可构建。

**legacy/\*.yaml_**（6 份停用）：与 `rmcs_core/src/hardware/legacy/*.cpp_` 对应（`DeformableInfantryOmni/OmniB`、`Flight`+`Px4VisionBridge`、`OmniInfantry`+`ui::Infantry`、`Sentry`+`interaction::SentryDecision`、`SteeringHeroLittle`）；v2 侧**没有**同名配置（在 `find /workspaces/reATRM -name "*omni*.yaml"` 下无命中）。

---

## 3. 差异清单

### 3.1 组件清单：v2 有而 v3 无 / v3 有而 v2 无

| 方向 | 组件类 | v2 出现机型 | v3 现状 | 影响/风险 |
|---|---|---|---|---|
| v2→无 | `rmcs_core::controller::chassis::OperateLogic`（`operate_logic`） | dual-infantry | 类不存在（grep `class OperateLogic` 无命中）；`dev/jgrobo-ize` 提交 `eeac05b` 已移除 `OmniWheelController` 一类 | 高：v2 dual-infantry 的速度/模式（comfort/shoot/hurt/home/normal + boost/spin）限速逻辑丢失 → 需迁到 v3 新骨架 |
| v2→无 | `rmcs_core::controller::chassis::OmniWheelController` | 4 机型 | v3 统一为 `JOmniWheelController`（`src/controller/chassis/j_omni_wheel_controller.cpp`） | 中：参数键与算法不同（§3.2） |
| v2→无 | `rmcs_core::controller::chassis::ChassisPowerController` | 4 机型 | v3 视机型用 `ChassisPowerController`（仍在 plugins.xml）或 `JChassisPowerController` | 中：dual-infantry v3 **完全没有**功率控制组件 |
| v2→无 | `rmcs_core::referee::app::ui::DualInfantry` / `DualSentry` / `MiniInfantry` | 4 机型 | v3 plugins.xml 只有 `ui::{Infantry, DeformableInfantry, Hero, Flight, AutoAimUi}` | 高：机型 UI 需在 v3 重建（依赖主题 14） |
| v2→无 | `autopilot::AutoPilotComponent` | dual-sentry | v3 无 autopilot 包/插件（grep `class AutoPilot` 无命中） | 中（提示词已声明 autopilot 不在范围） |
| v2→无 | `rmcs_core::hardware::PidTuneTool` | pid-tune-tool | 类不存在（grep 无命中）；也无 `config/pid-tune-tool.yaml` | 中：**PID 整定工具缺失**（§3.5） |
| v3→无 | `rmcs_core::controller::gimbal::SmcDualYawController` | — | dual-infantry 使用 | — |
| v3→无 | `rmcs_core::controller::chassis::JOmniWheelController` / `JChassisPowerController` | — | dual-sentry/mec-hero/mini-infantry 使用 | — |
| v3→无 | `rmcs::AutoAimCapturerComponent` / `AutoAimPlayerComponent` / `AutoAimVideoPlayerComponent` / `AutoAimRecorderComponent` | — | `auto_aim_test.yaml`（capturer 未进任何机型配置，见主题 17 §3.3） | 高 |
| v3→无 | `rmcs_core::referee::app::ui::AutoAimUi`、`referee::command::interaction::SentryDecision` | — | v3 新增 | — |
| v3→无 | `rmcs::navigation::Navigation` | — | `navigation_test.yaml`（包不在工作区） | **未核实** |

- dual-infantry 的组件数：**v2 = 23 / v3 = 13**（缺 10 条，其中 6 条是 referee 链 + UI、1 条 HeatController、1 条 AutoAim、1 条 ChassisPower、1 条 Diagnosis）。
- dual-sentry 的组件数：v2 = 24 / v3 = 22（差异仅 `ChassisPowerController→JChassisPowerController`、`OmniWheelController→JOmniWheelController`、移除 `AutoPilotComponent`、实例名 `dual_yaw_slover→dual_yaw_solver`）。

### 3.2 dual-infantry.yaml **逐键对比**

图例：`=` 键与值一致；`≠` 值不同；`—` 该侧不存在。

| 参数块 / 键 | v2 reATRM 值 | v3 jRMCS 值 | 结论 |
|---|---|---|---|
| `rmcs_executor.update_rate` | 1000.0 | 1000.0 | = |
| `rmcs_executor.components` | 23 项（§1.3） | 13 项（§2.3） | **≠ 缺 10 项** |
| `value_broadcaster.publish_divider` | — | 10 | v3 新增（`value_broadcaster.cpp:19` 默认 1） |
| `value_broadcaster.forward_list` | 14 项（`/gimbal/pitch/control_velocity, velocity_filtered, raw_angle, control_angle_error, /debug/solver/pitch, /chassis/power, …`） | 1 项：`/gimbal/yaw/control_angle_error` | **≠ 显著精简** |
| `dual_infantry_hardware.usb_pid_top_board` | `0x9d15` | — | **v2 键（libusb）** |
| `dual_infantry_hardware.usb_pid_bottom_board` | `0x34a4` | — | **v2 键** |
| `dual_infantry_hardware.board_serial_top_board` | — | `"D4-2EEA-E236-F8E4-62C8-9974-F3A5"` | **v3 键（序列号）** |
| `dual_infantry_hardware.board_serial_bottom_board` | — | `"D4-7C74-3B6F-7B82-B341-E573-0470"` | **v3 键** |
| `dual_infantry_hardware.bottom_yaw_motor_zero_point` | 6705 | 2687 | ≠ |
| `dual_infantry_hardware.top_yaw_motor_zero_point` | 7850 | 7916 | ≠ |
| `dual_infantry_hardware.pitch_motor_zero_point` | 35030 | 29364 | ≠ |
| `dual_infantry_hardware.imu_bias_{x,y,z}` | 3 / 8 / 2 | — | v3 硬件不再读取（`dual_infantry.cpp` 无 `imu_bias`；`ac99de6` 换 EKF IMU） |
| `gimbal_controller.shift_control_clamp` | 0.0045 | 0.0045 | = |
| `gimbal_controller.upper_limit` | -0.28 | -0.36 | ≠ |
| `gimbal_controller.lower_limit` | 0.35 | 0.66 | ≠ |
| `gimbal_controller.joystick_left_bias_{x,y}` | 0.0 / 0.0 | 0.0 / 0.0 | = |
| `gimbal_controller.depond_motors` | pitch/top_yaw/bottom_yaw | 同 | = |
| `gimbal_controller.yaw_upper_limit` / `yaw_lower_limit` | — | 1000000.0 / -1000000.0 | v3 新增 |
| `dual_yaw_controller.top_yaw_angle_kp/ki/kd` | 15.0 / 0.0 / 0.0 | 18.0 / 0.0 / 0.0 | ≠（kp） |
| `dual_yaw_controller.top_yaw_velocity_kp/ki/kd` | 2.0 / 0.01 / 0.0 | 1.0 / 0.0 / 0.0 | ≠ |
| `dual_yaw_controller.top_yaw_angle_output_min/max` | — | -6.0 / 6.0 | v3 新增（`SmcDualYawController` 读 `<name>_output_min/max`） |
| `dual_yaw_controller.top_yaw_velocity_output_min/max` | — | -1.5 / 1.5 | v3 新增 |
| `dual_yaw_controller.bottom_yaw_smc_c` | 10.0 | 3.0 | ≠ |
| `dual_yaw_controller.bottom_yaw_smc_k` | 1.5 | 1.5 | = |
| `dual_yaw_controller.bottom_yaw_smc_epsilon` | 1.0 | 0.3 | ≠ |
| `dual_yaw_controller.bottom_yaw_smc_phi` | 2.0 | 0.1 | ≠ |
| `dual_yaw_controller.bottom_yaw_inertia` | 2.0 | 0.5 | ≠ |
| `dual_yaw_controller.bottom_yaw_max_torque` | 16384.0 | 2.0 | **≠ 量级差 8000×（单位/归一化不同，需核实）** |
| `dual_yaw_controller.bottom_yaw_accel_ff_alpha` | — | 0.3 | v3 新增 |
| `dual_yaw_controller.top_yaw_angle_limit` | — | 0.9 | v3 新增 |
| `dual_yaw_solver` 块 | —（v2 `dual_yaw_solver.cpp` 不读任何参数，值硬编码） | `top_yaw_limit: 0.7854`（`get_parameter_or` 默认 π/4）、`spin_angle_bias: 0.0`（默认 0.6） | v3 新增可调参数（v2 组件清单里有 `DualYawSolver -> dual_yaw_slover` 但无参数块） |
| `chassis_controller.translational_velocity_limit` | 99.0 | — | v2 键 |
| `chassis_controller.angular_velocity_limit` | 25.0 | — | v2 键 |
| `chassis_controller.autopilot_spin_velocity_default` | 2.0 | — | v2 键（v3 无 autopilot） |
| `chassis_controller.translational_velocity_max` | — | 80.0 | **v3 键（`chassis_controller.cpp:339` 读取）** |
| `chassis_controller.angular_velocity_max` | — | 15.0 | **v3 键（`:340`）** |
| `operate_logic.*`（9 组 × 4 键） | comfort 10/15、shoot 0.5/2.5、hurt 2.5/1.2、home 1.0/1.0、normal 1.5/1.0（含 boost/spin） | — | **v2 独有，v3 无对应类与参数块** |
| `omni_wheel_controller.mass` | — | 15.0 | v3 新增（`j_omni_wheel_controller.cpp:33` 必读） |
| `omni_wheel_controller.moment_of_inertia` | — | 4.08 | v3 新增（`:34` 必读） |
| `omni_wheel_controller.chassis_radius_x/y` | 0.165 / 0.165 | 0.165 / 0.165 | = |
| `omni_wheel_controller.wheel_radius_` | 0.07 | — | v2 键名（v2 代码实际读 `wheel_radius` —— **遗留不一致**） |
| `omni_wheel_controller.wheel_radius` | — | 0.07 | v3 键名 |
| `omni_wheel_controller.friction_coefficient` | 0.6 | — | v2 键（v3 `JOmniWheelController` 不读） |
| `omni_wheel_controller.spinning_bias_adjust` | — | false | v3 新增 |
| `omni_wheel_controller.spinning_bais_coefficient` | -0.08 | -0.08 | = |
| `omni_wheel_controller.power_limit_default` | — | 45.0 | v3 新增（`get_parameter_or` 有默认） |
| `pitch_angle_err_pid_controller.enable` | `/gimbal/pitch/alive` | — | v3 未用（`ErrorPidController` 的 enable 接口） |
| `pitch_angle_err_pid_controller.kp/ki/kd` | 8.0 / 0.001 / 1.0 | 6.0 / 0.0 / 60.0 | ≠ |
| `pitch_angle_err_pid_controller.integral_min/max` | -5.0 / 5.0 | -5.0 / 5.0 | = |
| `pitch_velocity_pid_controller.enable` | `/gimbal/pitch/alive` | — | 同上 |
| `pitch_velocity_pid_controller.kp/ki/kd` | 2.5 / 0.05 / 45.0 | 1.2 / 0.0 / 15.0 | ≠ |
| `pitch_velocity_pid_controller.integral_min/max` | -25.0 / 25.0 | -2.0 / 2.0 | ≠ |
| `friction_wheel_controller.*` | wheels 2、600/600、soft 1.0 | 同 | = |
| `heat_controller.heat_per_shot / reserved_heat` | 10000 / 20000 | — | **v2 独有（组件也未加载）** |
| `bullet_feeder_controller.*` | 8 键（`bullets_per_feeder_turn 8 / shot_frequency 12 / safe 10 / eject 10 / eject_time .15 / deep 15 / deep_time .20 / single_shot_max_stop_delay 2`） | 同上 + `bullet_allowance_default: 100` | v3 新增 1 键 |
| `{left,right}_friction_velocity_pid_controller.*` | kp 0.003436926 / 0 / 0 | 同 | = |
| `bullet_feeder_velocity_pid_controller.measurement` | `/gimbal/bullet_feeder/velocity` | `/gimbal/bullet_feeder/velocity_filtered` | ≠（topic 名在 v3 是 filtered） |
| `bullet_feeder_velocity_pid_controller.kp/ki/kd` | 0.0 / 0.0 / 0.0 | 0.0 / 0.0 / 0.0 | = |
| `diagnosis.*` | `motors` 10 项 + `enable_motor_check/supercap_check/aim_check=true`、`enable_nav_check=false` | — | **v2 独有（组件也未加载）** |

### 3.3 launch 差异

| 差异点 | v2 | v3 | 影响/风险 |
|---|---|---|---|
| 包名 | `atrm_bringup` | `rmcs_bringup` | 迁移时全部替换 (`FindPackageShare`) |
| 节点数 | `rmcs_executor` + 自瞄 runtime（4 机型） | 仅 `rmcs_executor` | 高：自瞄采集/主循环改由插件承载，但 v3 **未把 capturer 加入任何机型配置**（主题 17 §3.3） |
| 自瞄启动条件 | 硬编码 `enable_vision_entities = ["dual-sentry","dual-infantry","mec-hero","mini-infantry"]` | 无 | v3 起 `auto_aim_test` 等也要跑自瞄，需新的选择机制（配置里列组件即可，无需改 launch） |
| 参数文件来源 | `share/atrm_bringup/config/<robot>.yaml` | `share/rmcs_bringup/config/<robot>.yaml` | 与 `set-robot`/`launch-rmcs` 约定一致（`Action-RMCS` 逻辑相同） |
| 其余 | `respawn=True, respawn_delay=1.0, output="log"` | 同 | = |
| 自动模式 | `robot:=auto.<name>` 分支（两者都有 `if is_automatic: pass` 空实现） | 同 | = |

### 3.4 包定义/CMake 差异

| 项 | v2 | v3 |
|---|---|---|
| 构建体系 | `ament_cmake_auto` + `ament_auto_package(INSTALL_TO_SHARE launch config)` | `ament_cmake` + 手写 `install(DIRECTORY config launch …)`（**重复两次**） |
| `cmake_minimum_required` | 3.8 | 3.10 |
| 依赖 | `rm_serial_driver / pb2025_robot_description / pb2025_nav_bringup / rm_vision_bringup` | 无这些；也**无** `rmcs_core`/`rmcs_auto_aim_v2` 的 `exec_depend` |
| `maintainer` | `MIDE<mide233@outlook.com>` | `Qzh<zihanqin2048@gmail.com>` |
| `license` | `TODO: License declaration` | `GPL-3.0-only` |

### 3.5 `pid-tune-tool` 是否缺失

**结论：v3 完全缺失。**
- v3 `rmcs_bringup/config/` 无 `pid-tune-tool.yaml`（6 份 config 已全列于 §2.1）。
- v3 `rmcs_core` 无 `PidTuneTool` 硬件类（`grep -rl "class PidTuneTool" src` 无命中；`rmcs_core/src/hardware/legacy/` 仅有 `deformable-infantry-omni(-b)/flight/omni_infantry/sentry/steering-hero-little-six-friction` 的 `.cpp_`）。
- v2 侧对应实现：`rmcs_core/src/hardware/model/pid_tune_tool.cpp` + `config/pid-tune-tool.yaml`（`PidController` 阶跃整定，参数 `motor_zero_point / velocity_step / step_time`）。
- 影响：**失去「电机速度环阶跃整定」的整定工具**，无法快速标定 `kp/ki/kd`；属独立工具，优先级 P1（非 P0 联调必需，但影响后续调参效率）。

### 3.6 v3 新增配置

| 配置 | 作用 | 与 v2 关系 |
|---|---|---|
| `config/auto_aim_test.yaml` | 离线自瞄测试（VideoPlayer + AutoAimComponent） | v2 无；替代 v2 的 `is_local_runtime` + `local_video` 配置方式 |
| `config/navigation_test.yaml` | 导航插件测试 | v2 无；插件包不在工作区（**未核实**） |
| `config/legacy/*.yaml_`（6 份） | 停用机型配置（deformable/flight/omni-infantry/sentry/steering-hero） | v2 无同名文件（v2 只保留 4 机型 + pid-tune-tool）；对应 `rmcs_core/src/hardware/legacy/*.cpp_` |

### 3.7 参数「配置 ↔ 代码」键名不一致（v3 侧遗留，波及多机型）

| 机型 | 现象 | 证据 | 影响 |
|---|---|---|---|
| dual-sentry（v3）`chassis_controller` | 仍是 v2 键 `translational_velocity_limit: 30.0 / angular_velocity_limit: 4.0 / autopilot_spin_velocity_default: 3.5` | v3 `chassis_controller.cpp:339-340` 只读 `translational_velocity_max` / `angular_velocity_max`；`grep` 全 `rmcs_core/src` 无任何组件读 `*_limit` / `autopilot_spin_velocity_default` | **中高**：底盘限速回落到代码默认（10.0 / 16.0），配置形同失效 |
| mec-hero / mini-infantry（v3）同上 | 同样是 `*_limit` / `autopilot_spin_velocity_default` | 同上 | 同上（dual-infantry 已改用新键 `80/15`，可作模板） |
| mec-hero / mini-infantry（v3）`omni_wheel_controller` | 用 `wheel_radius_`（带下划线）、无 `mass` / `moment_of_inertia` | v3 `j_omni_wheel_controller.cpp:33-37` 读 `mass, moment_of_inertia, chassis_radius_x, chassis_radius_y, wheel_radius`（`get_parameter` 非 `get_parameter_or`） | **高（待核）**：若缺键会抛 `ParameterNotDeclared`/类型异常，节点 `respawn` 循环；即使不抛也拿不到质量与惯量。v2 亦用 `wheel_radius_`，属**两版共有的遗留不一致**，缺 `mass` 则是 v3 新增需求 |
| dual-sentry（v3）`omni_wheel_controller` | 有 `mass`，缺 `moment_of_inertia` / `power_limit_default` | 同上 | 同上（`power_limit_default` 有 `get_parameter_or` 兜底） |
| 各机型（v3）`*_hardware.imu_bias_{x,y,z}` | 仍写在 dual-sentry/mec-hero/mini-infantry 配置里 | v3 `hardware/{dual_sentry,hero,mini_infantry}.cpp` 是否读取**未核实**（dual-infantry 已确认不读） | 低（多为无害冗余，需统一清理） |

> 该表属跨主题问题：算法/参数归属见主题 `08-控制器-底盘`、`07-硬件模型-其余机型`；本主题只记录**配置文件层面的不一致**。

### 3.8 机型参数差异（除 dual-infantry，简述）

- **dual-sentry**：v3 与 v2 键名/数值基本一致，仅硬件 board 标识由 `usb_pid_*`（`0xf18a/0x247f`）改为 `board_serial_*`（v3 值仍是 `"placeholder-top"/"placeholder-bottom"`，**未填实机序列号**）；`chassis_controller` 与 `omni_wheel_controller` 存在 §3.7 的遗留问题；`referee::app::ui::DualSentry` 组件缺失、`AutoPilotComponent` 缺失。
- **mec-hero**：v3 与 v2 差异集中在 `hero_hardware` 的 `usb_pid_*`→`board_serial_*`、`ui::Hero → referee_ui_hero` 组件缺失、`ChassisPowerController→JChassisPowerController`、`OmniWheelController→JOmniWheelController`；`hero_hardware` 的 `board_serial_*` 也是 `placeholder-*`。
- **mini-infantry**：同 mec-hero（`usb_pid_*`→`board_serial_*`、`ui::MiniInfantry` 缺失、`JOmniWheelController`/`JChassisPowerController`）；`mini_infantry_hardware.board_serial_*` 亦为 `placeholder-*`。

---

## 4. 迁移方案（以 v3 为骨架）

1. **补齐 `dual-infantry.yaml`（P0，首要）**：以 v3 `dual-sentry.yaml` 为模板把缺的组件补回（`referee::Status/Command/Interaction/Ui`、`rmcs::AutoAimComponent`、`shooting::HeatController`、`chassis::JChassisPowerController`、`diagnosis::Diagnosis`、一个 v3 存在的 UI 类），并补齐对应参数块（`heat_controller`、`diagnosis`、`auto_aim_component`…）。
   - UI：v3 无 `ui::DualInfantry`，短期用 `ui::Infantry`（`config/legacy/omni-infantry.yaml_` 用此），中期在主题 14 里新增 `DualInfantry` UI。
2. **接管 v2 的 `operate_logic` 语义（P0）**：v3 无此类；把「模式→速度上限」映射（comfort/shoot/hurt/home/normal + boost/spin）迁到 v3 底盘控制器（`ChassisController` 或 `JOmniWheelController`）或新组件，见主题 08。
3. **统一底盘参数键名（P0）**：把各机型 `translational_velocity_limit/angular_velocity_limit` 改为 `translational_velocity_max/angular_velocity_max`；删除 `autopilot_spin_velocity_default`；`omni_wheel_controller` 补齐 `mass/moment_of_inertia/wheel_radius/spinning_bias_adjust/power_limit_default`（dual-infantry 已是正确样板）。
4. **对齐自瞄组件与采集组件（P0）**：在需要自瞄的机型配置中加入 `rmcs::AutoAimCapturerComponent`（见主题 17 §4）。
5. **回收 v2 的 `value_broadcaster.forward_list`（P1）**：v3 各机型 forward_list 已不同，需按新接口名（`/auto_aim/*`、`/debug/*` 变化）重建，并确认 `publish_divider` 是否需要（dual-infantry 的 10 会显著降频）。
6. **移植 pid-tune-tool（P1）**：把 `rmcs_core/src/hardware/model/pid_tune_tool.cpp` 迁为 v3 `rmcs_core/src/hardware/pid_tune_tool.cpp`（或 `identification/` 下），注册进 `plugins.xml`，新增 `rmcs_bringup/config/pid-tune-tool.yaml`（键名按 v3 命名规范，`hardware::PidTuneTool -> hardware`）。注意 v3 板卡改用 `board_serial_*`。
7. **清理 launch 遗留与冗余（P2）**：`rmcs_bringup/CMakeLists.txt` 的重复 `install(DIRECTORY config launch …)` 删除一处；确认是否需要把 `rmcs_core` / `rmcs_auto_aim_v2` 加为 `exec_depend`。
8. **navigation_test 处置（P2）**：确认 `rmcs::navigation::Navigation` 的来源包；若无来源，则把 `navigation_test.yaml` 移入 `config/legacy/` 或补文档说明。
9. **补齐实机标识（P1）**：`dual-sentry/mec-hero/mini-infantry` 的 `board_serial_*` 目前是 `placeholder-*`，需按实机填写（否则板卡匹配失败）。

---

## 5. 执行步骤 checklist

- [ ] `dual-infantry.yaml`：补 `referee::*`（Status/Command/Interaction/Ui）、`auto_aim_component`、`heat_controller`、`chassis_power_controller`、`diagnosis`，并补齐对应参数块
- [ ] dual-infantry 底盘/云台参数键名与数值核对（`*_max`、`SmcDualYawController` 的 `output_min/max`、`top_yaw_angle_limit`、`bottom_yaw_max_torque` 量级）
- [ ] `operate_logic` 语义迁移（新组件或在 `ChassisController` 内实现）
- [ ] 全机型底盘键名统一（`*_limit`→`*_max`、删 `autopilot_spin_velocity_default`）
- [ ] 全机型 `omni_wheel_controller` 补齐 `mass/moment_of_inertia/wheel_radius/...`
- [ ] 需要自瞄的机型加入 `rmcs::AutoAimCapturerComponent`
- [ ] 重建各机型 `value_broadcaster.forward_list`（含 `/auto_aim/*`）
- [ ] 移植 `PidTuneTool` 硬件类 + `plugins.xml` 注册 + `config/pid-tune-tool.yaml`
- [ ] 填写 `dual-sentry/mec-hero/mini-infantry` 的 `board_serial_*` 实机值
- [ ] 处理 `navigation_test.yaml`（找来源包 / 归档）
- [ ] 删除 `bringup/CMakeLists.txt` 重复 `install`
- [ ] `clang-format` / `clang-tidy`（若动 C++）
- [ ] arm64 交叉编译通过 + 逐机型启动验证（无 respawn 循环）
- [ ] 更新本文件状态与 `docs-mig/README.md` 进度

---

## 6. 验收与验证

1. **构建**：`./.script/build-rmcs-cross --target-arch=arm64` 通过。
2. **逐机型启动**：`set-robot dual-infantry && launch-rmcs`；`docker logs`/`screen` 中应出现 `Starting RMCS on robot 'dual-infantry' -> dual-infantry.yaml`，随后所有组件注册成功；进程 **不** 反复重启。
3. **topic 巡检**：`rmcs-cli` / `foxglove` 检查
   - `/auto_aim/{should_control,control_direction,should_shoot}`（双步兵）
   - `/gimbal/{pitch,top_yaw,bottom_yaw}/{angle,control_torque}`、`/chassis/power`、`/chassis/supercap/voltage`
   - `/referee/id`、`/referee/shooter/initial_speed`
4. **参数生效核验**：用 `ros2 param get /rmcs_executor <name>` 或 `--ros-args --log-level debug` 确认 `chassis_controller` 的 `translational_velocity_max` 等**确为配置值**（而非代码默认 10.0/16.0）——这是 §3.7 的验收点。
5. **UI/功能**：双步兵 UI 显示自瞄状态环；云台右键可接管；左键/左拨杆可发弹。
6. **工具**：`pid-tune-tool` 机型可启动并完成一次速度环阶跃整定（移植后）。

---

## 7. 备注与未决问题

1. **`dual-infantry.yaml` 是当前最大的配置缺口**：v3 版只有 13 个组件、无 referee/UI/自瞄/诊断/热量/功率控制，且参数块缺 `operate_logic/heat_controller/diagnosis`；`git log -- rmcs_ws/src/rmcs_bringup/config/dual-infantry.yaml` 仅两次提交（`19ae676 first commit`、`ac99de6 feat(imu)…`），说明它是从上游带过来后**未像其它机型一样被补齐**。
2. **自瞄与 bringup 的耦合**：v3 把自瞄改为插件后，`launch` 不再需要每机型判断；但**采集组件尚未入配置**，主题 17 与 18 需同步执行（17 负责组件/接口，18 负责配置落地）。
3. **`RMCS_ROBOT_TYPE` 复用**：v2 自瞄配置通过 `RMCS_ROBOT_TYPE` 选机型（主题 17 §1.5），而 v3 自瞄配置改用 `AUTOAIM_CONFIG`/`config.yaml`。若希望「一个 `set-robot` 同时决定 bringup 与自瞄配置」，建议按主题 17 §4 方案 A 恢复 `RMCS_ROBOT_TYPE` 分支。
4. **未核实**：`rmcs_bringup/package.xml` 是否需要 `exec_depend` `rmcs_core`/`rmcs_auto_aim_v2`（当前无，靠 pluginlib 运行期查找）。
5. **未核实**：`rmcs::navigation::Navigation` 所属包（`navigation_test.yaml`），以及 `navigation_test` 是否在本次范围内（提示词已把导航列为「已知未覆盖项」）。
6. **未核实**：v3 `JOmniWheelController` 的 `get_parameter("mass", …)` 在配置缺键时的确切行为（抛异常 → `respawn` 循环，或回退默认值）；建议在主题 08 中通过一次 `robot:=mini-infantry` 实机/仿真启动确认。
7. **未核实**：`bottom_yaw_max_torque` 在 v2（16384.0）与 v3（2.0）相差约 8000 倍，需确认 v3 `SmcDualYawController` 的力矩单位/量纲（主题 09 云台）。
8. `dual-sentry/mec-hero/mini-infantry/dual-infantry` 的 `board_serial_*` 在 v3 多为 `placeholder-*`；`dual_infantry_hardware` 的是真实序列号（`D4-…`），可作填写参考。
9. v2 config 路径与文件行号均以 `/workspaces/reATRM/src/bringup/config/*.yaml` 为准；v3 以 `/workspaces/RMCS/rmcs_ws/src/rmcs_bringup/config/*.yaml` 为准（本次核对时 v3 bringup 工作区**无未提交改动**，`git status --short` 仅显示 `m rmcs_ws/src/rmcs_auto_aim_v2` 与 `?? docs-mig/`）。

---

## 迁移状态（阶段二，2026-10-19）

**已完成。** 重建文件：`rmcs_ws/src/rmcs_bringup/config/dual-infantry.yaml`。
- 组件 13 → **24**：补齐 referee 链/UI/heat/chassis_power/diagnosis/auto_aim(capturer)。
- 修正 v3 键名（`translational_velocity_max`/`angular_velocity_max`、`wheel_radius`、补 `mass`/`moment_of_inertia`）；补全 `SmcDualYawController` 必读积分限；去掉死键 `enable:`。
- 零点/`imu_bias`/限速采用 v2 值（带 TODO）；`operation_mode`/`operate_logic` 按决策删除。
- 验证：`yaml.safe_load` 通过，24 组件。

---

## 迁移状态（阶段三，2026-10-19）

- **`dual-sentry.yaml` / `mini-infantry.yaml` / `mec-hero.yaml`**：组件补齐到 **24 条**（补机型 UI + `rmcs::AutoAimCapturerComponent`，新增 `auto_aim_capturer` 参数块 `use_hardware_sync: false`）；统一修正：
  - `chassis_controller`：`translational_velocity_limit`→`translational_velocity_max`、`angular_velocity_limit`→`angular_velocity_max`，删 `autopilot_spin_velocity_default`。
  - `gimbal_controller`：补 `yaw_upper_limit: 1000000.0` / `yaw_lower_limit: -1000000.0`（`SimpleGimbalController` 必读，否则启动抛异常）。
  - `omni_wheel_controller`：`wheel_radius_`→`wheel_radius`、`spinning_bais_coefficient`→`spinning_bias_coefficient`，删除 `friction_coefficient`，补 `moment_of_inertia: 4.08`、`spinning_bias_adjust: false`。
  - PID 块：删除死键 `enable:`；`bullet_feeder_velocity_pid_controller.measurement` 由 `/gimbal/bullet_feeder/velocity_filtered` 改为 `/gimbal/bullet_feeder/velocity`。
  - `value_broadcaster`：清掉 v3 已不存在的 `/debug/solver/*`、`/debug/aim/*`、`/debug/feeder/*`、`/referee/chassis/current_hp`（→`/referee/current_hp`）等悬空项。
  - `board_serial_*` 保留 placeholder + TODO。
- **新增 `pid-tune-tool.yaml`（方案 B）**：替代 v3 已删除的 `PidTuneTool`，在 `hardware::DualInfantry` 上运行 `identification::SweptFrequencyController`（`/gimbal/pitch` 扫频）与 `identification::StaticTorqueTestController`（`/gimbal/top_yaw` 静态扭矩），`debug::ValueCollector` 落 CSV，`ValueBroadcaster` 转发调参信号。
- **验证**：三份机型 config 各 24 组件、`pid-tune-tool.yaml` 5 组件，`yaml.safe_load` 均通过；坏模式 grep 干净；交叉编译全绿。
- **提交**：`6efb71f`（三份机型 config）、`4402b23`（pid-tune-tool）。
