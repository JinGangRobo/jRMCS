# 04-消息与接口-rmcs_msgs与interface

- 状态：待办
- 优先级：P0（枚举/头文件差异直接决定移植代码能否编译；`ChassisMode` / `RobotId` 语义变化影响底盘与裁判系统行为）
- 影响机型：全部；重点 dual-infantry（`ChassisMode`/`OperateMode`）、dual-sentry（新增 `SentryEvent`）、mec-hero / mini-infantry（UI 与 operate_mode）
- 依赖主题：01-仓库布局与包结构、03-组件运行时-rmcs_executor、08-控制器-底盘、12-诊断、14-裁判系统-UI
- 验收方式：① 交叉编译 arm64 通过（消息为头文件式，编译即验证）；② `grep` 复核所有 `rmcs_msgs::` 引用点均可解析；③ dual-infantry 实机日志中`RobotColor` 不再出现"未知来源被判为红方"；④ 若采纳 `OperateMode` 回迁，验证 ASSIST/MANUAL 切换与 UI 显示一致

## 1. v2（reATRM）现状

v2 的"消息与接口"由**两个包**组成，二者都在 `/workspaces/reATRM/src/interface/` 下：

### 1.1 `rmcs_msgs`（头文件式枚举/结构）

路径：`/workspaces/reATRM/src/interface/rmcs_msgs`

- `include/rmcs_msgs/*.hpp`（14 个）：`autopilot.hpp`、`chassis_mode.hpp`、`full_robot_id.hpp`、`game_stage.hpp`、`gimbal_mode.hpp`、`keyboard.hpp`、`mouse.hpp`、`operate_mode.hpp`、`robot_color.hpp`、`robot_id.hpp`、`serial_interface.hpp`、`shoot_mode.hpp`、`shoot_status.hpp`、`switch.hpp`
- `msg/*.msg`（6 个）：`ArmorPlate.msg`、`ArmorPlateArray.msg`、`GameStatus.msg`、`Point2f.msg`、`RobotPose.msg`、`RobotStatus.msg`
- `CMakeLists.txt`（`project(rmcs_msgs)`，`-std=c++20`）：关键事实 —— 只收集 `MSG_PATH` 变量，**没有调用 `rosidl_generate_interfaces`**（全仓库 `grep -rn rosidl_generate_interfaces` 仅命中 `interface/CMakeLists.txt`）；`package.xml` 虽声明 `<member_of_group>rosidl_interface_packages</member_of_group>`，但 `.msg` 实际**未生成** ROS 接口（构建产物中无 `rmcs_msgs/msg/*.hpp`，`未核实` 是否有其他生成途径）。
- v2 代码只使用**头文件中的枚举/结构**，全仓库无 `rmcs_msgs/msg` 或 `rmcs_msgs::msg` 引用。

### 1.2 `interface` 包（包名 `atrm_interfaces`）

路径：`/workspaces/reATRM/src/interface/interface`

- 目录名 `interface`，但 `CMakeLists.txt:2` `project(atrm_interfaces)`、`package.xml` `<name>atrm_interfaces</name>`（**目录名与包名不一致**）。
- `msg/CmdGimbal.msg`：`float64 pitch` / `float64 yaw` / `bool fire`
- `msg/SupercapInfo.msg`：`float64 super_cap_voltage` / `bool is_super_cap_work`
- `CMakeLists.txt:13-21` 真正调用 `rosidl_generate_interfaces(${PROJECT_NAME} ... DEPENDENCIES std_msgs geometry_msgs)`；依赖 `geometry_msgs`。
- **全仓库无任何 C++ 消费方**：`grep -rn "SupercapInfo\|CmdGimbal" /workspaces/reATRM` 只命中该包自身的 `CMakeLists.txt:14-15`。超容实际是通过 `librmcs`/串口在 `rmcs_core/src/hardware/device/supercap.hpp` 内自行解析（不经 ROS 消息）。

## 2. v3（jRMCS）现状

路径：`/workspaces/RMCS/rmcs_ws/src/rmcs_msgs`

- `include/rmcs_msgs/*.hpp`（19 个）：`board_clock.hpp`、`camera_frame.hpp`、`camera_frame_raw.hpp`、`chassis_mode.hpp`、`full_robot_id.hpp`、`game_stage.hpp`、`gimbal_mode.hpp`、`imu_snapshot.hpp`、`keyboard.hpp`、`mouse.hpp`、`rmcs_msgs.hpp`、`robot_color.hpp`、`robot_id.hpp`、`sentry_event.hpp`、`serial_interface.hpp`、`shoot_condiction.hpp`、`shoot_mode.hpp`、`shoot_status.hpp`、`switch.hpp`
- `msg/*.msg`（6 个）：与 v2 **同集合、逐字节相同**
- `CMakeLists.txt` / `package.xml`：与 v2 **逐字节相同**（同样没有 `rosidl_generate_interfaces`）
- **没有** `interface` / `atrm_interfaces` 包（`grep -rn atrm_interfaces rmcs_ws/src` 无命中）
- v3 独有内容要点：
  - `board_clock.hpp`：`struct BoardClock`（4 MHz 计时基准，`is_steady = true`），供 `imu_snapshot.hpp` 与 `board_clock_lifter.hpp` 使用。
  - `imu_snapshot.hpp`：`struct ImuSnapshot { Eigen::Quaterniond orientation; Eigen::Vector3d gyro_body; BoardClock::time_point timestamp; }`。
  - `camera_frame.hpp` / `camera_frame_raw.hpp`：1440×1080 原始帧缓冲 + `opencv_cvt_color_code` + 曝光/收帧/发布三个时间戳（视觉子系统用）。
  - `sentry_event.hpp`：`enum class SentryEvent : std::uint8_t`（SWITCH_POSE_* / CONFIRM_* / EXCHANGE_* / ACTIVATE_ENERGY_CORE / COUNT），被 `rmcs_core/src/referee/command/interaction/sentry_decision.cpp:23` 使用。
  - `shoot_condiction.hpp`（注：文件名拼写为 `condiction`）：`enum class ShootCondiction : uint8_t { FRICTION_WAITING, PRELOADING, SHOOT, JAM, FIRED }`，被 `rmcs_core/src/referee/app/ui/hero.cpp:13` 使用。
  - `rmcs_msgs.hpp`：聚合头 + `constexpr to_string(...)` 重载（GameStage/ChassisMode/GimbalMode/RobotColor/ArmorID/RobotId/ShootMode/Switch/FullRobotId）+ `std::formatter` 特化（以 `RMCS_MSGS_HAS_STD_FORMAT` / `__cpp_lib_format >= 201907L` 守卫）。

## 3. 差异清单

### 3.1 `rmcs_msgs/include` 逐文件（`diff -u` 实测）

| 文件（v2 路径 → v3 路径） | 结论 | 说明 |
|---|---|---|
| `autopilot.hpp` | **v3 删除** | v3 无对应文件 |
| `operate_mode.hpp` | **v3 删除** | v3 无对应文件 |
| `chassis_mode.hpp` | **内容变化** | 枚举重排 + 新增 3 值 + 新增 2 个 helper |
| `robot_id.hpp` | **内容变化** | `color()` / `id()` 的 UNKNOWN 处理 + `noexcept` |
| `full_robot_id.hpp` | 相同（md5 一致） | — |
| `game_stage.hpp` | 相同（md5 一致） | — |
| `gimbal_mode.hpp` | 相同（md5 一致） | — |
| `keyboard.hpp` | 相同（md5 一致） | — |
| `mouse.hpp` | 相同（md5 一致） | — |
| `robot_color.hpp` | 相同（md5 一致） | `UNKNOWN=0/RED=1/BLUE=2` 不变 |
| `serial_interface.hpp` | 相同（md5 一致） | — |
| `shoot_mode.hpp` | 相同（md5 一致） | — |
| `shoot_status.hpp` | 相同（md5 一致） | — |
| `switch.hpp` | 相同（md5 一致） | — |
| `board_clock.hpp` | **v3 新增** | 4 MHz 板级时钟 |
| `camera_frame.hpp` | **v3 新增** | 视觉帧（含 IMU 快照） |
| `camera_frame_raw.hpp` | **v3 新增** | 视觉原始帧 |
| `imu_snapshot.hpp` | **v3 新增** | IMU 快照结构 |
| `rmcs_msgs.hpp` | **v3 新增** | 聚合头 + `to_string` + `std::formatter` |
| `sentry_event.hpp` | **v3 新增** | 哨兵决策事件（dual-sentry 用） |
| `shoot_condiction.hpp` | **v3 新增** | 发射状态机枚举 |

### 3.2 `chassis_mode.hpp`（唯一"破环式"枚举变化）

v2（`/workspaces/reATRM/.../rmcs_msgs/include/rmcs_msgs/chassis_mode.hpp`）：

```cpp
enum class ChassisMode : uint8_t {
    AUTO = 0, SPIN = 1, STEP_DOWN = 2, LAUNCH_RAMP = 3, AUTO_PILOT = 4,
};
```

v3（`/workspaces/RMCS/.../rmcs_msgs/include/rmcs_msgs/chassis_mode.hpp`）：

```cpp
enum class ChassisMode : uint8_t {
    AUTO, SPIN_SLOW, SPIN_FAST, STEP_DOWN, LAUNCH_RAMP, ALIGNMENT, ALIGNMENT_POWERED, CLIMB,
};
constexpr auto is_powered(ChassisMode) noexcept;   // ALIGNMENT_POWERED / LAUNCH_RAMP / CLIMB
constexpr auto is_spining(ChassisMode) noexcept;   // SPIN_SLOW / SPIN_FAST
```

逐条：

| 差异点 | v2 | v3 | 影响 | 风险 |
|---|---|---|---|---|
| `SPIN` 拆分 | `SPIN = 1` | `SPIN_SLOW`(=1) / `SPIN_FAST`(=2) | v2 的"自旋"单一模式变为两档 | 移植 v2 底盘代码必须决定档位；v3 用 `is_spining()` 统一判断 |
| 数值重排 | `STEP_DOWN=2, LAUNCH_RAMP=3, AUTO_PILOT=4` | `STEP_DOWN=3, LAUNCH_RAMP=4, ALIGNMENT=5, ALIGNMENT_POWERED=6, CLIMB=7` | 枚举不再显式编号，值与 v2 不同 | 已核实**不存在** `static_cast<ChassisMode>(int)` 或 YAML 中的数字映射（`grep` 无命中），故仅源码层影响，无 ABI/配置风险 |
| `AUTO_PILOT` 删除 | `AUTO_PILOT = 4` | 无 | v2 `chassis_controller.cpp:133,135,163,181,241` 引用该值（自动驾驶/导航支线，**不在迁移范围**） | 移植 v2 `chassis_controller` 时必须删除或改写 `AUTO_PILOT` 分支，否则**编译失败** |
| 新增 `ALIGNMENT` / `ALIGNMENT_POWERED` / `CLIMB` | 无 | 有 | 供哨兵爬坡/取力对齐等 v3 功能 | v2 侧无对应逻辑；若"扬长避短"只移植 v2 功能，可不动 |
| 新增 helper | 无 | `is_powered()` / `is_spining()` | 统一判断，减少散落 switch | v2 代码散落的 `mode == ChassisMode::SPIN` 需改 `is_spining(mode)` |

使用统计（编译影响面）：v2 使用 `ChassisMode::` 共 5 处 `SPIN`、3 处 `AUTO`、5 处 `STEP_DOWN`、5 处 `LAUNCH_RAMP`、2 处 `AUTO_PILOT`；v3 已迁移为 `SPIN_FAST`/`SPIN_SLOW`/`ALIGNMENT(_POWERED)`/`CLIMB`。

### 3.3 `robot_id.hpp`

v2：

```cpp
constexpr RobotColor color() const { return value_ & 0x40 ? RobotColor::BLUE : RobotColor::RED; }
constexpr ArmorID id() const { return value_ > 100 ? static_cast<ArmorID>(value_ - 100) : static_cast<ArmorID>(value_); }
```

v3：

```cpp
constexpr RobotColor color() const noexcept {
    if (value_ == Value::UNKNOWN) return RobotColor::UNKNOWN;
    return value_ & 0x40 ? RobotColor::BLUE : RobotColor::RED;
}
constexpr ArmorID id() const noexcept {
    if (value_ == Value::UNKNOWN) return ArmorID::Unknown;
    return value_ > 100 ? static_cast<ArmorID>(value_ - 100) : static_cast<ArmorID>(value_);
}
```

| 差异点 | 影响 | 风险 |
|---|---|---|
| `color()` 对 `UNKNOWN(0)` 的返回值：v2 = `RED`，v3 = `UNKNOWN` | 裁判系统状态/UI 里"机器人 ID 未知"不再被当成红方 | **行为差异**：若移植 v2 代码在"ID 未知"时依赖 `color()==RED` 做分支（如默认红方 UI 配色），需显式处理 `UNKNOWN`。双车 infantry/哨兵的 `RobotColor` 来自 `full_robot_id` 拼装，实际很少为 0，风险中低 |
| `id()` 对 `UNKNOWN` 显式 `ArmorID::Unknown`（v2 因 0 值巧合也是 `Unknown`） | 无语义变化 | 低 |
| 新增 `noexcept` | 无行为变化 | 低 |
| `ArmorID` / `RobotId::Value` 具体数值 | 与 v2 相同（md5 一致） | 无 |

### 3.4 删除的 `autopilot.hpp` / `operate_mode.hpp`

`autopilot.hpp`（v2）：

```cpp
enum class PilotDiag : uint8_t { BAD_RELOCATION, WARNING, FATAL, READY, STARTING, OFFLINE, SLAMING };
enum class NavMode : uint8_t { SLAM, RELOCATION, UNKNOWN };
```

使用点（v2）：`rmcs_core/src/controller/diagnosis.cpp:10,103,106,109`（依赖 `PilotDiag`）、`src/core/utils/system_ctrl/autopilot_client/include/fields.hpp:3,7,8` 与 `.../autopilot_client/src/component.cpp:51,99-102,111,119,134-135,165`。

`operate_mode.hpp`（v2）：

```cpp
enum class OperateMode : uint8_t { MANUAL = 0, ASSIST = 1 };
```

使用点（v2，较多）：
- UI：`rmcs_core/src/referee/app/ui/{hero.cpp:12,119,223, dual_infantry.cpp:12,120,224, dual_sentry.cpp:12,119,224, mini_infantry.cpp:12,119,223}`
- 底盘：`rmcs_core/src/controller/chassis/chassis_controller.cpp:16,140,290`、`.../chassis/operate_logic.cpp:10,83,88,90,200`

v3 现状：`grep -rn "OperateMode\|ASSIST" rmcs_ws/src` **无任何命中**（仅 `chassis_climber_controller.cpp:331` 的无关日志文本 "MANUAL_SUPPORT_RETRACT"）；`grep -rln "autopilot\|PilotDiag\|NavMode" rmcs_ws/src` 仅命中 `rmcs_bringup/config/dual-sentry.yaml`（配置中的舵机/命名巧合，见主题 18）。

| 差异点 | v2 | v3 | 影响 | 风险 |
|---|---|---|---|---|
| `PilotDiag` / `NavMode` | 有 | 无 | 影响主题 12（diagnosis）；`autopilot_client` 本身**不在迁移范围**（见 `prompts/00-prompt-v1.md` 第 2 节） | 若移植 v2 `diagnosis.cpp`，须同时决定是否回迁 `autopilot.hpp` 或删除该诊断分支 |
| `OperateMode` | 有 | 无 | 影响主题 08（底盘 assist/manual 逻辑）与主题 14（UI 显示） | 若"扬长避短"需保留双车 ASSIST 语义，需**新增回该头文件**并在 v3 底盘/UI 中重新接线；否则 v2 底盘与 UI 代码移植会**编译失败** |

### 3.5 `msg/*.msg` 与 `interface` 包

| 差异点 | v2 | v3 | 影响 |
|---|---|---|---|
| `rmcs_msgs/msg/*.msg`（6 个） | 存在，未生成 | 同内容（md5 全一致），未生成 | 无 |
| `interface`（`atrm_interfaces`）包 | 存在，`CmdGimbal` / `SupercapInfo`，真正生成 | **不存在** | v2 内无消费者，`supercap` 走串口；**可不迁移**。若未来需要 ROS 消息形式的云台指令/超容信息，需新建包并处理目录名/包名不一致问题 |
| `CMakeLists.txt` / `package.xml`（rmcs_msgs） | 见 1.1 | 逐字节相同 | 无 |

### 3.6 平台/编译相关

- v3 新增头文件使用 `<eigen3/Eigen/Geometry>`（`camera_frame.hpp` / `camera_frame_raw.hpp` / `imu_snapshot.hpp`）——与 v3 `rmcs_core` 其它文件一致的 include 风格；交叉编译需 Eigen 头可见（现状已满足）。
- `rmcs_msgs.hpp` 的 `std::formatter` 特化需 C++23/libstdc++ 的 `std::format`；v2 无此类用法，不影响 v2 代码移植（除非移植代码 include 该聚合头）。

## 4. 迁移方案（以 v3 为骨架）

1. **不新增/不删除 v3 `rmcs_msgs` 文件**（除下列 2、3 项决策）。v3 的枚举语义作为唯一真源；移植 v2 代码时改代码，不改 v3 枚举。
2. **`OperateMode` 决策（建议：回迁头文件）**：若决定保留 v2 的 ASSIST/MANUAL 功能（底盘 + UI），最小改动为在 v3 新增
   `rmcs_msgs/include/rmcs_msgs/operate_mode.hpp`（内容与 v2 逐字节相同），并在主题 08/14 中把 v3 对应组件接线到该接口；不改 `rmcs_msgs.hpp` 聚合头（避免引入 `to_string` 需求）。
   - 若不回迁：移植 v2 底盘/UI 时删除 `operate_mode_` 输入与 ASSIST 分支（功能降级，需用户确认）。
3. **`autopilot.hpp` 决策（建议：不回迁，改 diagnosis）**：`autopilot_client` 不在迁移范围，v2 `diagnosis.cpp` 中对 `PilotDiag` 的依赖在移植时**删除或改为可选输入**（`InputInterface<PilotDiag>` 的接线在 v3 无 provider）。若用户要求保留，则回迁该头文件并在主题 12 说明。
4. **`ChassisMode`**：不动 v3 枚举；移植 v2 底盘代码时按下表机械替换：
   `ChassisMode::SPIN` → `SPIN_SLOW` / `SPIN_FAST`（或 `is_spining()`）、`AUTO_PILOT` → 删除（导航支线）或映射为 `AUTO`、其余同名值直接沿用。
5. **`RobotId`**：不动 v3 实现；移植代码中凡有"ID 未知即红方"的隐含假设，改为显式判断 `RobotColor::UNKNOWN`。
6. **`interface` / `atrm_interfaces`**：**不迁移**（v2 无消费者）。在 `docs-mig/README.md` 与主题 05 中留档为"v2 独有、无功能影响"。
7. **`msg/*.msg`**：保持现状（两侧都不生成）。若将来需要 ROS 化，单独开主题（需补 `rosidl_generate_interfaces`），本次不做。

## 5. 执行步骤 checklist

- [ ] 决策并记录：`OperateMode` 是否回迁（04-1，阻塞主题 08/14）
- [ ] 决策并记录：`autopilot.hpp` / `PilotDiag` 是否回迁（04-2，阻塞主题 12）
- [ ] 若回迁：新增 `rmcs_msgs/include/rmcs_msgs/operate_mode.hpp`（与 v2 逐字节一致）（04-3）
- [ ] 移植 v2 底盘代码时替换 `SPIN` → `SPIN_SLOW`/`SPIN_FAST`，删除 `AUTO_PILOT`（04-4，联动主题 08）
- [ ] 复核移植代码中所有 `RobotId::color()` 的 UNKNOWN 分支（04-5，联动主题 13/14）
- [ ] 复核移植代码是否 include 了 v3 不存在的头（`autopilot.hpp` / `operate_mode.hpp`）（04-6）
- [ ] 确认不引入 `atrm_interfaces` 依赖（04-7）
- [ ] 交叉编译 arm64 全量通过（04-8）

## 6. 验收与验证

- 编译即验证（`rmcs_msgs` 为纯头文件）：`./.script/build-rmcs-cross --target-arch=arm64 --packages-skip rmcs_auto_aim_v2 --executor sequential`。
- 引用点复核：`grep -rn "rmcs_msgs/autopilot\|rmcs_msgs/operate_mode\|ChassisMode::SPIN\b\|AUTO_PILOT" rmcs_ws/src` 应无命中（或仅命中注释）。
- 枚举一致性：`grep -rn "ChassisMode::" rmcs_ws/src/rmcs_auto_aim_v2`（视觉侧也用该枚举）应与 `rmcs_core` 用法一致。
- 行为验证（实机/离线不可验）：dual-infantry 下 `RobotColor` 为 UNKNOWN 时 UI 不再显示红方配色（若采纳 04-5）。
- 文档一致性：`rmcs_msgs.hpp` 的 `to_string(ChassisMode)` switch 必须覆盖全部 8 个枚举值（v3 现状已覆盖，改动枚举时需同步）。

## 7. 备注与未决问题

- v2 `rmcs_msgs` 的 `.msg` 文件**从未被生成**（无 `rosidl_generate_interfaces`），因此"v2 用 ROS 消息通信"的假设不成立：v2/v3 的进程内通信全部走 `rmcs_executor` 命名接口（`rmcs_msgs` 头文件类型）。这一点决定了本主题的迁移量很小。
- `atrm_interfaces` 的目录名（`interface`）与包名（`atrm_interfaces`）不一致，且无消费者；**未核实**是否曾用于早期真机调试（无 git 历史证据可查）。
- `shoot_condiction.hpp` 文件名拼写错误（应为 `condition`）——属 v3 现状，本阶段不改名（改名会引发大量 include 变更），仅记录。
- v3 `sentry_event.hpp` / `shoot_condiction.hpp` / `camera_frame*.hpp` / `imu_snapshot.hpp` 是否需要在 v2 侧对应实现：取决于主题 17（视觉）与 13/14（哨兵）的迁移结论，本文件不做功能判断。
- `robot_id.hpp` 的 `color()` 行为变化（UNKNOWN: RED→UNKNOWN）是否会影响 `rmcs_core` 中 v3 已有逻辑：**未逐处核实**，需在主题 13 一并检查。
