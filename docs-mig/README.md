# docs-mig —— jRMCS(v3) ← reATRM(v2) 迁移档案

本目录记录 **jRMCS（v3，新架构）** 与 **reATRM（v2，旧架构）** 的全量差异，
以及**以 v3 为骨架、把 v2 可用功能移植进 v3** 的迁移计划。

- **目标仓库（可写）**：`/workspaces/RMCS`（jRMCS）
- **参照仓库（只读）**：`/workspaces/reATRM`（reATRM）
- **当前阶段**：阶段三「执行」——**P1 其余机型全链路**（dual-sentry / mini-infantry / mec-hero），含 07 硬件 / 14 UI / 18 config / `pid-tune-tool` 弃用替代。
  阶段二（dual-infantry，P0）已完成主题 06/08/09/10/14/17/18 的代码改动，arm64 交叉编译全绿；详见「执行记录」。

---

## 目录结构

```
docs-mig/
├── README.md                       # 本文件：索引 + 进度
├── prompts/
│   └── 00-prompt-v1.md             # 迭代后的任务提示词（决策依据）
├── plans/
│   ├── 00-master-plan.md           # 总迁移计划：基线/批次/依赖/验收/风险
│   └── 01-diff-matrix.md           # 差异总览矩阵与依赖图
└── topics/                         # 每主题一个差异档案（统一模板）
    ├── 01-仓库布局与包结构.md
    ├── 02-工程链路-构建与容器与CI.md
    ├── 03-组件运行时-rmcs_executor.md
    ├── 04-消息与接口-rmcs_msgs与interface.md
    ├── 05-硬件栈-librmcs与设备驱动.md
    ├── 06-硬件模型-dual_infantry.md          # P0
    ├── 07-硬件模型-其余机型.md
    ├── 08-控制器-底盘.md
    ├── 09-控制器-云台.md
    ├── 10-控制器-发射.md
    ├── 11-控制器-PID与滤波与数学工具.md
    ├── 12-诊断-diagnosis.md
    ├── 13-裁判系统-Status与Command.md
    ├── 14-裁判系统-UI.md
    ├── 15-TF与描述.md
    ├── 16-通用工具-rmcs_utility.md
    ├── 17-视觉子系统-rmcs_auto_aim_v2.md
    └── 18-bringup-配置与launch.md
```

---

## 决策速览（来自 `prompts/00-prompt-v1.md`）

| 项 | 决策 |
|---|---|
| 比对范围 | 运行时全链路 + 工程链路 + 视觉子系统（**不含**导航/SLAM/autopilot） |
| 迁移方向 | **v3 架构为骨架**，把 v2 可用功能/调参/控制器移植进 v3 |
| C++ 标准 | **维持现状分层**（核心包 23 / 其余 20），仅移植处适配 |
| operate_mode/autopilot | **完全放弃操作辅助**，不回迁 |
| hikcamera/serial | 目标环境/sysroot 已提供，**无需处理** |
| 功率控制 | 用 **`JChassisPowerController`**（v2 等价） |
| 实机事实 | **先按 v2 值迁移**，加 TODO，实机再标定 |
| 自瞄 | 必须接入；用 v3 的 **`AUTOAIM_CONFIG`** 机制 |
| 范围 | **只做 dual-infantry(P0)**；其余机型 / pid-tune-tool 仅出文档 |
| 验收标准 | arm64 交叉编译全绿 + 静态/结构校验 + `clang-format` |
| 粒度 | 按主题分批，每批一次 arm64 交叉编译，不中途 review |

### 阶段三决策（2026-10-19）

| 项 | 决策 |
|---|---|
| 主方向 | **P1 其余机型全链路**（dual-sentry / mini-infantry / mec-hero） |
| 顺序/范围 | **三机型一次做完**，允许改代码 |
| 单测 | **不新建 gtest 基建**；验收 = 交叉编译 + `clang-tidy`/静态核验 |
| `pid-tune-tool` | **方案 B：弃用**，改用 `identification::*` + `debug::ValueCollector` 并补可运行 config |
| 提交 | **分主题提交到本地仓库** |

---

## 主题清单与状态

状态取值：`待办` / `进行中` / `已完成`。

| 主题 | 优先级 | 影响机型 | 批次 | 状态 |
|---|---|---|---|---|
| 01 仓库布局与包结构 | P2 | 全局 | 6 | 待办（不在本次代码范围） |
| 02 工程链路-构建与容器与CI | P1 | 全局 | 6 | 待办（不在本次代码范围） |
| 03 组件运行时-rmcs_executor | P0 | 全局 | 1 | 已完成（v3 为超集，无需改动） |
| 04 消息与接口-rmcs_msgs与interface | P0 | 全局 | 1 | 已完成（operate_mode/autopilot 按决策放弃） |
| 05 硬件栈-librmcs与设备驱动 | P0 | 全机型 | 2 | 已完成（驱动已下沉，dual-infantry 无缺件） |
| 06 硬件模型-dual_infantry | **P0** | dual-infantry | 2 | **已完成**（referee serial/supercap/gy614/imu_bias/CAN2/电流环） |
| 07 硬件模型-其余机型 | P1 | sentry/hero/mini | 2 | 未改动（仅文档） |
| 08 控制器-底盘 | **P0** | 全机型 | 3 | 已完成（J* 控制器就位；OperateLogic 按决策放弃） |
| 09 控制器-云台 | P0 | 全机型 | 3 | **已完成**（simple_gimbal_controller 补 v2 特性） |
| 10 控制器-发射 | P0 | 全机型 | 3 | **已完成**（heat 输出 + 接 HeatController） |
| 11 控制器-PID与滤波与数学工具 | P1 | 多机型 | 3 | 已完成（配置层处理；`enable` 不复刻） |
| 12 诊断-diagnosis | P1 | 全机型 | 3 | **已完成**（接入 Diagnosis，蜂鸣器无法移植） |
| 13 裁判系统-Status与Command | P1 | 全机型 | 4 | 已完成（v3 已含，接入 config） |
| 14 裁判系统-UI | **P0** | 全机型 | 4 | **已完成**（新建 ui::DualInfantry + 注册） |
| 15 TF与描述 | P2 | sentry | 4 | 已完成（无需改动） |
| 16 通用工具-rmcs_utility | P1 | 全局 | 1 | 已完成（v3 为超集，无需改动） |
| 17 视觉子系统-rmcs_auto_aim_v2 | P0 | 需自瞄机型 | 5 | **已完成**（接入 capturer + 硬同步输出） |
| 18 bringup-配置与launch | **P0** | dual-infantry | 5 | **已完成**（dual-infantry.yaml 重建，24 组件） |

---

## 进度表（执行阶段更新）

| 批次 | 主题 | 编译 | 单测/静态 | 格式 | 实机 | 状态 |
|---|---|---|---|---|---|---|
| Batch0 | 基线/决策 | ✅ | — | — | ☐ | 完成（基线 7 包全绿，决策已记录） |
| Batch1 | 03/04/16 | ✅ | ✅ | ✅ | — | 完成（v3 超集，无需改动） |
| Batch2 | 05/06/07 | ✅ | ✅ | ✅ | ☐ | 完成（06 已改；05/07 无需改） |
| Batch3 | 08/09/10/11/12 | ✅ | ✅ | ✅ | ☐ | 完成（09 控制器 + 10/12 接入） |
| Batch4 | 13/14/15 | ✅ | ✅ | ✅ | — | 完成（14 新建 UI） |
| Batch5 | 17/18 | ✅ | ✅ | ✅ | ☐ | 完成（17 接入 + 18 重建 config） |
| Batch6 | 01/02 + 收口 | ✅ | — | — | — | 未做（按决策不在本次代码范围） |

> 说明：`单测/静态` 以 `clang-format --dry-run -Werror`（改动文件）+ `nm` 插件符号核查 + `yaml.safe_load` 结构校验代替；
> `实机` 一列留空（☐）表示需上真机标定，相关 TODO 已写入代码与 config。

---

## 执行记录（阶段二，只做 dual-infantry）

> 基线：`dev/jgrobo-ize`，改动前 `build-rmcs-cross --target-arch=arm64 --packages-skip rmcs_auto_aim_v2` 已 7 包全绿。

### 改动文件清单

| 文件 | 主题 | 性质 |
|---|---|---|
| `rmcs_ws/src/rmcs_core/src/hardware/dual_infantry.cpp` | 06/17 | 改 |
| `rmcs_ws/src/rmcs_core/src/referee/app/ui/dual_infantry.cpp` | 14 | **新增** |
| `rmcs_ws/src/rmcs_core/src/controller/gimbal/simple_gimbal_controller.cpp` | 09 | 改 |
| `rmcs_ws/src/rmcs_core/src/controller/shooting/heat_controller.cpp` | 10 | 改 |
| `rmcs_ws/src/rmcs_core/plugins.xml` | 14 | 改（注册 UI、去掉重复项） |
| `rmcs_ws/src/rmcs_bringup/config/dual-infantry.yaml` | 18 | 重建 |

### 要点

- **06 硬件模型**：接回 `/referee/serial`（BottomBoard UART1）、`Jsupercap`(28.5)、`Gy614` 摩擦轮温度（TopBoard UART2）、`imu_bias_x/y/z`；
  pitch 回到 **CAN2**（收 0x213/发 0x3）；yaw 恢复 `kGM6020` 电流环；TopBoard IMU 恢复恒等映射。均带 TODO 待实机标定。蜂鸣器因 v3 SDK 无 API 未移植。
- **14 UI**：按 v2 `dual_infantry` UI 适配 v3 重写（受伤方向/操作辅助/中心区/电池弧等 v3 已无接口的部分已去掉）。
- **09/10/12**：`simple_gimbal_controller` 补 `depond_motors`/`shift_control_clamp`/`joystick_left_bias`，`joystick_sensitivity` 回 v2 的 0.004；
  `HeatController` 新增 `/gimbal/shooter/heat` 输出；config 接入 `HeatController` 与 `Diagnosis`。
- **17 自瞄**：`dual_infantry.cpp` 新增 `/gimbal/auto_aim/imu_snapshot`（必开）与可选的 `/gimbal/auto_aim/exposure_signal`（`camera_sync_gpio_channel` 控制）；
  config 接入 `AutoAimCapturerComponent`（`use_hardware_sync: false`，因曝光信号未接线）。
- **18 config**：`dual-infantry.yaml` 组件由 13 → **24**，补齐 referee 链/UI/heat/chassis_power/diagnosis/auto_aim；修正 v3 键名
  （`*_max`、`wheel_radius`、`mass`/`moment_of_inertia`）、补全 `SmcDualYawController` 必读积分限、去掉死键 `enable:`。
- **决策落实**：放弃 `OperateLogic`/`operate_mode`/`autopilot`；功率控制用 `JChassisPowerController`；实机量先按 v2 值迁移。

### 验证

- `./.script/build-rmcs-cross --target-arch=arm64 --packages-skip rmcs_auto_aim_v2 --executor sequential` → **EXIT=0，7 包全绿，无 error/warning**。
- `nm -C build-cross-arm64/rmcs_core/librmcs_core.so | grep ui::DualInfantry` → 含 `registerPlugin<referee::app::ui::DualInfantry>`。
- `clang-format --dry-run -Werror`：新增/改动文件通过（`dual_infantry.cpp` 的存量违规与 HEAD 一致，非本次引入）。
- `yaml.safe_load`：`dual-infantry.yaml` 结构有效，24 组件。

---

## 使用方式

1. 先读 `prompts/00-prompt-v1.md` 与 `plans/00-master-plan.md`（`§9.1` 为已确认决策）。
2. 按 `plans/01-diff-matrix.md` 的批次与依赖执行。
3. 每个 `topics/*.md` 是一个可独立执行的迁移单元：按其 §4 方案与 §5 checklist 落地，按 §6 验收。
4. 每完成一个主题：更新该文件顶部状态 + 本 README 的进度表。

---

## 约束

- **reATRM 只读**：不得修改 `/workspaces/reATRM` 下任何文件。
- 阶段一（建档）不修改代码；阶段二（执行）已获用户确认，**仅限 dual-infantry**。
- 交叉编译验证日志写入可写目录（`/tmp/opencode/` 或 `rmcs_ws/log-cross-arm64/`）。
- 注意 v2(`c++20`) 与 v3(`CMAKE_CXX_STANDARD 23`) 的标准差异，以及 librmcs 集成方式差异（子模块 vs FetchContent SDK）。

---

## 已核实的重要更正（相对初始假设）

- 「v3 的 `integral/clear_integral` 是新增」**不成立**：两侧 `pid_calculator.hpp` 均有（见主题 11）。
- 「TF 描述了 `RawImu` 是 v3 新增」**不成立**：`tf_description.hpp` 两版字节一致，`RawImu` 早已存在；v3 真实新增是 `sentry_description.hpp`（见主题 15）。
- 「底盘扭矩分配/自旋平移未迁移」**不成立**：已完整搬入 `JOmniWheelController`（见主题 08）。
