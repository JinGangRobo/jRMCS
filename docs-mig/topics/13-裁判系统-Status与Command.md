# 裁判系统 — Status 与 Command

- 状态：待办
- 优先级：P0
- 影响机型：全机型；其中 sentry 相关（`SentryInfo` / `SentryCommand`）影响 dual-sentry，`MapCommand` 影响带导航/地图指令的机型
- 依赖主题：14（裁判系统 UI，UI 消费 status 输出）、15（TF 与描述，`/referee/position/*` 与自瞄/导航相关）
- 验收方式：交叉编译通过 + 逐接口名/类型比对（下文清单）+ foxglove 观察 `/referee/**` topic

> 比对基准：
> - v2（reATRM，只读）：`/workspaces/reATRM/src/core/utils/system_ctrl/rmcs_core/src/referee/`
> - v3（jRMCS）：`/workspaces/RMCS/rmcs_ws/src/rmcs_core/src/referee/`
>
> 方法：对同名文件执行 `diff -u`，并逐行核对 `register_input/register_output`、`field.hpp` 结构体、命令分发表。所有结论均可按路径 + 行号复查。

---

## 1. v2（reATRM）现状

### 1.1 文件清单与规模

| 文件 | v2 行数 | 说明 |
|---|---|---|
| `status.cpp` | 258 | 状态解析组件 `rmcs_core::referee::Status` |
| `status/field.hpp` | 92 | 裁判系统下行数据体结构 |
| `frame.hpp` | 27 | 帧头/帧体 |
| `command.cpp` | 107 | 上行发送组件 `rmcs_core::referee::Command` |
| `command/field.hpp` | 64 | `Field`（可调用写包） |
| `command/interaction.cpp` | 64 | `command::Interaction` 仲裁 |
| `command/interaction/header.hpp` | 14 | 交互包头 |
| `command/interaction/ui.cpp` | 167 | `command::interaction::Ui` 打包 |
| `command/interaction/communicate.cpp` | 0（空文件） | 未实现 |
| `command/interaction/sentry_decision.cpp` | 不存在 | — |
| `command/map_marker.cpp` | 0（空文件） | 未实现 |
| `command/text_display.cpp` | 0（空文件） | 未实现 |

### 1.2 `Status` 输出的接口（v2）

`status.cpp:30–51`：

```
/referee/game/stage            GameStage
/referee/game/remain_time      double
/referee/game/center_area      uint8_t
/referee/id                    rmcs_msgs::RobotId
/referee/shooter/cooling       int64_t
/referee/shooter/heat_limit    int64_t
/referee/chassis/power_limit   double
/referee/chassis/buffer_energy double
/referee/chassis/output_status bool
/referee/robot/hurt            status::HurtData
/referee/chassis/current_hp    double
/referee/robots/hp             status::GameRobotHp
/referee/shooter/bullet_allowance       uint16_t
/referee/shooter/42mm_bullet_allowance  uint16_t
/referee/shooter/initial_speed double
/referee/shooter/shoot_timestamp double
/referee/robots/rmul_rfid      uint8_t
```

### 1.3 命令分发表（v2，`status.cpp:109–133`）

`0x0001` game_status / `0x0003` game_robot_hp / `0x0101` event_data / `0x0201` robot_status /
`0x0202` power_heat / `0x0203` robot_position（空实现）/ `0x0206` hurt_data /
`0x0207` shoot_data / `0x0208` bullet_allowance / `0x0209` robot_rfid_position /
`0x020B` game_robot_position（空实现）。

### 1.4 `status/field.hpp` 结构（v2）

- `GameStatus{ game_type:4, game_stage:4, stage_remain_time, sync_timestamp }`
- `EventData{ uint32_t event_data }`（裸 32bit，未拆字段）
- `GameRobotHp{ robot_1..robot_4, reserved, robot_7, robot_outpost, robot_base }`
- `RobotStatus`（无 `bullet_speed_limit`，无 `reserved` 位）
- `PowerHeatData{ reserved_0, reserved_1, float reserved_2, buffer_energy, shooter_17mm_1_barrel_heat, shooter_42mm_barrel_heat }`
- `BulletAllowance{ bullet_allowance_17mm, bullet_allowance_42mm, remaining_gold_coin, projectile_allowance_fortress }`
- `Rfid{ rfid_status, rfid_status_2 }`
- `GameRobotPosition{ hero_x..infantry_4_y, reserved_0/1 }`

### 1.5 `Command`（上行，v2 = v3）

`command.cpp` 完全相同（107/107 行，无 diff）：输入 `/referee/serial`、`/referee/command/interaction`、
`/referee/command/map_marker`、`/referee/command/text_display`；按 25Hz(`0x0301`) / 1Hz(`0x0307`) /
3Hz(`0x0308`) 发送。`map_marker.cpp`、`text_display.cpp` 在 v2/v3 都是**空文件**，故这两个上行通道**均未实现**。

`command/interaction.cpp` 相同（64/64）：输入 `/referee/command/interaction/sentry_decision`、
`/referee/command/interaction/communicate`、`/referee/command/interaction/ui`，按该优先级择一输出
`/referee/command/interaction`。注意：v2 的 `interaction.cpp` **已经**声明了 `sentry_decision` 输入，
只是没有人产出它。

---

## 2. v3（jRMCS）现状

### 2.1 文件清单与规模 / diff 概览

| 文件 | v2 行 | v3 行 | `diff -u` 行数 | 结论 |
|---|---|---|---|---|
| `status.cpp` | 258 | 412 | 380 | **大改** |
| `status/field.hpp` | 92 | 161 | 190 | **大改** |
| `frame.hpp` | 27 | 27 | 0 | 相同 |
| `command.cpp` | 107 | 107 | 0 | 相同 |
| `command/field.hpp` | 64 | 64 | 0 | 相同 |
| `command/interaction.cpp` | 64 | 64 | 0 | 相同 |
| `command/interaction/header.hpp` | 14 | 14 | 0 | 相同 |
| `command/interaction/ui.cpp` | 167 | 168 | 9 | 仅文件末尾换行差异 |
| `command/interaction/communicate.cpp` | 0 | 0 | 0 | 空 |
| `command/interaction/sentry_decision.cpp` | 不存在 | 243 | — | **v3 独有** |
| `command/map_marker.cpp` | 0 | 0 | 0 | 空 |
| `command/text_display.cpp` | 0 | 0 | 0 | 空 |

### 2.2 `Status` 输出的接口（v3，`status.cpp:30–105`）

```
/referee/game/stage                          GameStage
/referee/game/stage_remain_time              uint16_t     <- 由 v2 的 /referee/game/remain_time 改名+改类型
/referee/game/sync_timestamp                 uint64_t     <- 新增
/referee/event/ally_big_energy_activation_status    uint8_t  <- 新增
/referee/event/ally_small_energy_activation_status  uint8_t  <- 新增
/referee/event/ally_fortress_occupation_status      uint8_t  <- 新增
/referee/dart/latest_hit_target_total_count  uint8_t      <- 新增
/referee/id                                  rmcs_msgs::RobotId
/referee/shooter/cooling                     int64_t
/referee/shooter/heat_limit                  int64_t
/referee/chassis/power_limit                 double
/referee/chassis/power                       double       <- 新增
/referee/chassis/buffer_energy               double
/referee/chassis/output_status               bool
/referee/sentry/posture                      uint8_t      <- 新增（默认 3）
/referee/sentry/is_powered                   bool         <- 新增
/referee/sentry/is_disengaged                bool         <- 新增
/referee/sentry/can_rebirth_free             bool         <- 新增
/referee/sentry/can_rebirth_gold             bool         <- 新增
/referee/sentry/rebirth_gold_cost            uint16_t     <- 新增
/referee/robots/hp                           status::GameRobotHp
/referee/ally/hero_hp                        uint16_t     <- 新增
/referee/ally/engineer_hp                    uint16_t     <- 新增
/referee/ally/infantry_1_hp                  uint16_t     <- 新增
/referee/ally/infantry_2_hp                  uint16_t     <- 新增
/referee/ally/outpost/hp                     uint16_t     <- 新增
/referee/ally/base/hp                        uint16_t     <- 新增
/referee/enemy/outpost/hp                    uint16_t     <- 新增
/referee/enemy/base/hp                       uint16_t     <- 新增
/referee/damage_difference                   int16_t      <- 新增
/referee/current_hp                          uint16_t     <- 由 /referee/chassis/current_hp(double) 改名+改类型
/referee/position/x|y|angle                  double       <- 新增（0x0203）
/referee/shooter/bullet_allowance            uint16_t
/referee/shooter/42mm_bullet_allowance       uint16_t
/referee/shooter/fortress_17mm_bullet_allowance uint16_t  <- 新增
/referee/remaining_gold_coin                 uint16_t     <- 新增
/referee/shooter/initial_speed               float        <- 类型由 double 改为 float
/referee/shooter/shoot_timestamp             double
/referee/map_command/target_position_x|y     double       <- 新增
/referee/map_command/keyboard                uint8_t      <- 新增
/referee/map_command/target_robot_id         uint8_t      <- 新增
/referee/map_command/source                  uint16_t     <- 新增
/referee/map_command/received_timestamp      double       <- 新增
/referee/map_command/event/target_position_x|y double     <- 新增（去重后事件）
/referee/map_command/event/keyboard          uint8_t      <- 新增
/referee/map_command/event/target_robot_id   uint8_t      <- 新增
/referee/map_command/event/source            uint16_t     <- 新增
/referee/map_command/event/timestamp         double       <- 新增
/referee/map_command/event/sequence          uint64_t     <- 新增
```

v3 已移除：`/referee/game/center_area`、`/referee/robot/hurt`、`/referee/robots/rmul_rfid`、
`/referee/chassis/current_hp`（改名为 `/referee/current_hp`）、`/referee/game/remain_time`（改名）。

### 2.3 命令分发表（v3，`status.cpp:159–185`）

```
0x0001 -> update_game_status
0x0003 -> update_game_robot_hp
0x0101 -> update_event_data
0x0105 -> update_dart_info          <- 新增
0x0201 -> update_robot_status
0x0202 -> update_power_heat_data
0x0203 -> update_robot_position     <- 由空实现改为解析 RobotPosition
0x0206 -> update_hurt_data          <- 变为空实现（保留函数但不再写输出）
0x0207 -> update_shoot_data
0x0208 -> update_bullet_allowance
0x020D -> update_sentry_info        <- 新增
0x0303 -> update_map_command        <- 新增
```
v3 不再处理 `0x0209`（Rfid）与 `0x020B`（GameRobotPosition）。

### 2.4 `status/field.hpp` 结构（v3）

- `GameStatus`：字段 `game_stage` → **`game_progress`**（同一 `:4` 位域）。
- `EventData`：改为具名位域，含 `ally_small_energy_activation_status:2`、
  `ally_big_energy_activation_status:2`、`ally_fortress_occupation_status:2` 等；
  新增 `static_assert(sizeof(EventData) == 4)`。
- `GameRobotHp`：字段重命名 `robot_1..robot_4 → ally_1..ally_4`，`reserved → damage_difference(int16_t)`，
  新增 `enemy_outpost_hp`、`enemy_base_hp`；`robot_7 → ally_7_robot_hp`、
  `robot_outpost → ally_outpost_hp`、`robot_base → ally_base_hp`。
- `RobotStatus`：新增 `float bullet_speed_limit` 与 `reserved:5`，新增
  `static_assert(sizeof(RobotStatus) == 17)`。
- `PowerHeatData`：`reserved_0` 被删除，`reserved_2` 由 `float` 改 `uint16_t`，
  `shooter_17mm_1_barrel_heat → shooter_17mm_barrel_heat`。
- `BulletAllowance`：`bullet_allowance_17mm → projectile_allowance_17mm`、
  `bullet_allowance_42mm → projectile_allowance_42mm`（`remaining_gold_coin`/`projectile_allowance_fortress` 保留）。
- 删除 `Rfid`（v2 第 41 行）、`GameRobotPosition`（v2 第 80 行）。
- **保留** `HurtData`（v3 第 85 行）、`RobotPosition`（v3 第 79–83 行）、`ShootData`、`BulletAllowance`。
- 新增 `DartInfo`（`static_assert(sizeof==3)`）、`MapCommand`（`sizeof==12`）、
  `SentryCommand`（`sizeof==4`，含 `enum class Posture`）、`SentryInfo`（`sizeof==14`）。
- 注意：v3 **保留了 `HurtData` 结构**，但 `status.cpp` 已不再解析/输出它（见 D4）。

### 2.5 `sentry_decision.cpp`（v3 独有，243 行）

类 `rmcs_core::referee::command::interaction::SentryDecision`
（`register` 见 `sentry_decision.cpp:87–95`）：

- 输入：`/referee/id`、`/rmcs_navigation/sentry_events`(可缺省)、`/referee/sentry/posture`(可缺省)、
  `/referee/current_hp`(可缺省)、`/referee/event/ally_big_energy_activation_status`(可缺省)、
  `/referee/sentry/can_rebirth_free`(可缺省)。
- 输出：`/referee/command/interaction/sentry_decision`（`Field`）。
- 行为：把 `rmcs_msgs::SentryEvent` 事件按 `kEventPriority`（`sentry_decision.cpp:67–80`）转成
  `status::SentryCommand`（`command_id = 0x0120`，`sentry_decision.cpp:156`），并用姿态/hp/能量核心反馈
  `verify_feedback()`（`:197–236`）清除已确认请求；`ACTIVATE_ENERGY_CORE` 处留有
  `// FIXME: 加 5s 超时销毁`（`:225`）。
- 依赖的 `rmcs_msgs::SentryEvent`（`/workspaces/RMCS/rmcs_ws/src/rmcs_msgs/include/rmcs_msgs/sentry_event.hpp`）
  是 **v3 独有**（v2 无该头文件）。
- 它消费的 `/referee/current_hp`、`/referee/sentry/*`、`/referee/event/ally_big_*` 都是 v3 `status.cpp` 新增输出，
  即 `SentryDecision` 与 v3 `Status` **强耦合**。

### 2.6 插件注册

v3 `rmcs_core/plugins.xml:57–67`：`referee::Status`、`referee::Command`、`command::Interaction`、
`command::interaction::Ui`、`command::interaction::SentryDecision`、
`app::ui::{Infantry,DeformableInfantry,Hero,Flight,AutoAimUi}`。

v2 `plugins.xml:101–125`：`referee::Status`、`referee::Command`、`command::Interaction`、
`command::interaction::Ui`、`app::ui::{Infantry,Hero,DualSentry,DualInfantry,MiniInfantry}`。
**v2 没有 `SentryDecision`**；v3 没有 `DualSentry/DualInfantry/MiniInfantry`。

### 2.7 机型 config 接线

| 机型 config | v2 | v3 |
|---|---|---|
| `dual-infantry.yaml` | 有 `Status/Command/Interaction/Ui` + `app::ui::DualInfantry` | **整块 referee 组件缺失**（仅硬件+控制器） |
| `dual-sentry.yaml` | 有全套 + `app::ui::DualSentry` | 有 `Status/Command/Interaction/Ui`，**无任何 `app::ui::`**；**未接 `SentryDecision`** |
| `mini-infantry.yaml` | 有全套 + `app::ui::MiniInfantry` | 有 `Status/Command/Interaction/Ui`，**无任何 `app::ui::`** |
| `mec-hero.yaml` | 有全套 + `app::ui::Hero` | 有 `Status/Command/Interaction/Ui`，**无任何 `app::ui::`** |

（v3 中出现过的 `app::ui::*` 接线目前只存在于 `rmcs_bringup/config/legacy/*.yaml_`，见主题 14。）

---

## 3. 差异清单

| # | 差异点 | v2 做法 | v3 做法 | 影响 | 风险 |
|---|---|---|---|---|---|
| D1 | 比赛阶段字段名 | `GameStatus.game_stage` | `GameStatus.game_progress`（同位域） | 仅结构体成员名变化，语义不变（`status.cpp:190`） | 若有按名访问的外部代码需同步；本仓内无 |
| D2 | 剩余时间接口 | `/referee/game/remain_time`，`double` | `/referee/game/stage_remain_time`，`uint16_t` | **改名 + 改类型**，任何订阅旧名的组件失效 | 中 |
| D3 | 当前血量接口 | `/referee/chassis/current_hp`，`double` | `/referee/current_hp`，`uint16_t` | **改名 + 改类型** | 中 |
| D4 | 受击数据 | `/referee/robot/hurt`（`HurtData`）+ `hurt_data_watchdog_` | **输出接口被移除**；`0x0206` 仍分发到 `update_hurt_data()`，但函数体为空；`HurtData` 结构体仍在 `status/field.hpp:85` | v2 的 dual/hero UI 依赖它做受击方向提示（见主题 14） | **高** |
| D5 | 中心区状态 | `/referee/game/center_area`（`uint8_t`） | **移除** | v2 UI 的 `StatusBar::update_dynamic_part(center_status)` 数据源消失 | **高** |
| D6 | RMUL RFID | `/referee/robots/rmul_rfid`；处理 `0x0209` | **移除**，不再处理 `0x0209` | 依赖 RFID 的逻辑需另寻数据源 | 中 |
| D7 | 机器人位置 | `0x0203`/`0x020B` 均为空实现 | `0x0203` 解析 `RobotPosition` → `/referee/position/{x,y,angle}` | v3 新增；可用 | 低 |
| D8 | 事件数据 | 裸 `uint32_t event_data`，代码位移取 `center_area` | 具名位域 + `static_assert`；新增大小/大能量激活、堡垒占领 | 结构体可读性/安全性提升；语义字段变化 | 低 |
| D9 | 机器人血量 | `GameRobotHp.robot_1..`/`robot_outpost`/`robot_base` | 重命名 `ally_*` + `enemy_*` + `damage_difference`，并拆出 `/referee/ally|enemy/**` 独立输出 | v3 新增细粒度接口 | 低 |
| D10 | 弹丸允许量字段名 | `bullet_allowance_17mm/42mm` | `projectile_allowance_17mm/42mm` + `projectile_allowance_fortress` | 结构体成员改名 | 低 |
| D11 | 发射初速类型 | `OutputInterface<double>` | `OutputInterface<float>` | 订阅方若写死 `double` 会绑定失败 | 中 |
| D12 | 哨兵信息 | 无 | 新增 `SentryInfo`(14B) + `/referee/sentry/*` 输出（`posture = posture + (is_powered?3:0)`） | v3 新增；是 `SentryDecision` 的前置 | 低 |
| D13 | 哨兵决策 | 无（仅 `interaction.cpp` 预留输入） | 新增 `SentryDecision`（`0x0120`，`rmcs_msgs::SentryEvent`） | dual-sentry 的哨兵决策能力 | 中（`ACTIVATE_ENERGY_CORE` 超时未做） |
| D14 | 地图指令 | 无 | 新增 `0x0303 → MapCommand` + `/referee/map_command/**`（含去重事件流） | 新增能力 | 低（`event/sequence` 单调自增，未见消费方） |
| D15 | 上行发送通道 | `map_marker.cpp`/`text_display.cpp` 空 | 同为空 | `Command` 的 `0x0307/0x0308` 永不触发 | 低 |
| D16 | `interaction::Ui` | 167 行 | 168 行（仅末尾换行） | 无功能差异 | 无 |
| D17 | 机型接线 | 4 机型都接 `app::ui::*` | **无任何机型接 `app::ui::*`**；dual-infantry 连 referee 组件都缺 | 见主题 14 | **高** |

---

## 4. 迁移方案（以 v3 为骨架）

v3 的 `Status/field.hpp/SentryDecision/MapCommand` 是**升级方向**，应保留 v3 实现；迁移目标是补齐
「v2 有、v3 被删/被改名」的数据面，并接线。

1. **保留 v3 全部新增接口**（D7–D14），不回退。
2. **补齐 D4 受击数据**：在 `status.cpp` 恢复 `0x0206 → update_hurt_data()` 的解析与 `HurtData` 输出。
   两条路线：
   - (推荐) 复用 v3 的裁判协议：重新在 `status/field.hpp` 增加 `HurtData` 结构，`register_output("/referee/robot/hurt", ...)`，并恢复 5 tick 看门狗逻辑（v2 `status.cpp:40/87/180`）。同时把受击方向所需的 TF 查询保留（主题 14）。
   - (替代) 若不想恢复旧名，新增语义等价的新接口（如 `/referee/event/hurt`），但 UI 需同步改名。
3. **补齐 D5 中心区状态**：v3 `EventData` 已拆位域，可从中导出中心区占领状态并新增输出
   （如 `/referee/game/center_area` 或 `/referee/event/center_gain_point_occupied`）。
   `status_bar.hpp`（v3）仍带 `center_status` 形参，UI 只需数据源。
4. **D2/D3 改名**：全仓 grep 旧名（`/referee/game/remain_time`、`/referee/chassis/current_hp`）确认无残留消费方；
   v3 内部已无残留（`sentry_decision.cpp` 已用新名）。
5. **D11 类型统一**：`sentry_decision`/UI 订阅 `/referee/shooter/initial_speed` 时用 `float`。
   v3 UI 的 `StatusRing::update_bullet_speed` 已删除（见主题 14），迁移后无消费者。
6. **接线**：在各机型 `rmcs_bringup/config/*.yaml` 恢复 `referee::Status/Command/Interaction/interaction::Ui`
   与机型对应 `app::ui::*`（dual-sentry 另加 `interaction::SentryDecision`）。dual-infantry 需整块补回。

---

## 5. 执行步骤 checklist

- [ ] 建立接口对照表（v2 名 → v3 名/类型），全仓 grep 旧接口名确认消费方
- [ ] D4：恢复受击数据输出（`HurtData` + `/referee/robot/hurt` + 看门狗），或确定替代接口名
- [ ] D5：补中心区/占领点输出，供 UI `status_bar` 使用
- [ ] D11：确认 `initial_speed` 使用 `float`
- [ ] D13：`dual-sentry.yaml` 接线 `SentryDecision`
- [ ] D17：4 机型 config 接线 referee 组件（dual-infantry 需补 `Status/Command/Interaction/Ui`）
- [ ] `rmcs_core/plugins.xml` 与源文件一致性核对（新增/删除组件后）
- [ ] 交叉编译：`./.script/build-rmcs-cross --target-arch=arm64 --packages-skip rmcs_auto_aim_v2 --executor sequential`
- [ ] `clang-format --dry-run -Werror` 于改动文件

---

## 6. 验收与验证

- **编译**：上述交叉编译命令通过（重点看 `rmcs_core`）。
- **接口名**：`ros2 topic list | grep /referee`，核对第 2.2 节清单；订阅方无 `未连接` 告警。
- **插件**：`nm -C install-cross-arm64/lib/librmcs_core.so | grep -i 'referee.*Status\|SentryDecision'`。
- **行为**：foxglove 观察 `/referee/game/stage`、`/referee/current_hp`、`/referee/sentry/posture`；
  受击/中心区（D4/D5）若恢复，用回放/注入帧验证。

---

## 7. 备注与未决问题

- `command/interaction/communicate.cpp`、`map_marker.cpp`、`text_display.cpp` 在 v2/v3 均为 0 字节空文件；
  `Command` 中对应上行链路为「预留未实现」。
- `SentryDecision` 中 `ACTIVATE_ENERGY_CORE` 反馈清理注明 `FIXME: 加 5s 超时销毁`（`sentry_decision.cpp:225`）。
- v3 `status.cpp` 顶部改为 `#include <cstring>`（供 `memcpy`）并去掉了 `<rclcpp/logging.hpp>`；
  `map_command` 处理使用 `memcpy` + `memcmp` 去重（`status.cpp` `update_map_command`）。
- 「v3 新增 RawImu link」的说法**未在裁判系统内成立**；RawImu 属 TF/描述主题（两者文件相同），详见主题 15。
- 未核实项：无（`HurtData`、`RobotPosition` 均已确认存在于 v3 `status/field.hpp`；v3 缺的是
  `/referee/robot/hurt` 的 `register_output` 与解析逻辑，恢复 D4 只需改 `status.cpp`，无需补结构体）。
