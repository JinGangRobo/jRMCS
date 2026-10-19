# 00 总迁移计划（Master Plan）

> 依据：`prompts/00-prompt-v1.md`（迭代后的任务提示词）。
> 本计划是**阶段二（执行）**的总纲；具体步骤在各 `topics/*.md`。

---

## 1. 目标与原则

**目标**：把 reATRM（v2）中**经实车验证可用**的车型功能 / 控制器 / 调参 / 工具，移植进 jRMCS（v3）的新架构，
使 v3 在**保持新架构骨架**的前提下，达到「具体车型能工作」的水平。

**原则**
1. **v3 架构为骨架**：不改上游组件/插件模型、不改 `rmcs_executor` 的对外约定（除非为兼容 v2 语义而**新增**能力）。
2. **增量移植、不删除既有功能**：v3 独有能力（ADRC、Deformable、Climber、SMC、事件接口、统计等）保留；v2 的可用逻辑以**新增/补齐**方式并入。
3. **优先可运行**：先让 P0（dual-infantry）能编译、能加载、能跑通链路，再做参数校正与打磨。
4. **可追溯**：每步改动对应 `topics/*.md` 中一条差异与一条 checklist。
5. **reATRM 只读**；jRMCS 改动只在其仓库内。

---

## 2. 范围

| 纳入 | 说明 |
|---|---|
| 运行时全链路 | executor、msgs/接口、设备驱动、硬件模型、控制器、裁判系统/UI、TF 描述 |
| 工程链路 | `.script`、CMake、交叉编译工具链、Dockerfile、CI、devcontainer、部署/同步 |
| 视觉子系统 | `rmcs_auto_aim_v2` 及其与主链路接口 |

| 不纳入 | 说明 |
|---|---|
| 导航 / SLAM / `autopilot_client` | 用户明确排除；在各处作为「已知未覆盖项」留档（如 `navigation_test.yaml`、`AutoPilotComponent`、`rmcs_msgs/autopilot.hpp`） |

**机型优先级**：**dual-infantry = P0**；dual-sentry / mini-infantry / mec-hero / pid-tune-tool = P1/P2（仍记录差异，执行排在 P0 之后）。

---

## 3. 基线（Baseline，开始前必须建立）

1. **编译基线**：执行
   ```bash
   ./.script/build-rmcs-cross --target-arch=arm64 --packages-skip rmcs_auto_aim_v2 --executor sequential
   ```
   记录 error/warning；确认 `rmcs_core` 现状是否已可编译（主题 06 已核实 dual_infantry.cpp 于 2026-10-17 编译通过）。
2. **产物时效**：`install-cross-arm64/lib/librmcs_core.so` 时间戳为 2026-10-15，早于 10-17 的 `.o` → **验证前必须重新构建/安装**，避免复用旧 `.so`。
3. **事实确认清单（需用户/实机确认，见 §9）**：板卡 serial、pitch 物理 CAN 通道、yaw 电机控制模式、IMU 坐标映射、零点来源、自瞄相机内参来源。
4. **运行基线**：`set-robot dual-infantry && launch-rmcs`（占用真实硬件；离线环境仅做静态核对）。

---

## 4. 跨主题关键阻塞项（Cutting Concerns）

> 这些是**多主题共用**的前置问题，必须最先决策/统一，否则各主题互相返工。

| # | 阻塞项 | 涉及主题 | 现状 | 决策/动作 |
|---|---|---|---|---|
| CC1 | **C++ 标准不统一** | 01/02 | v3 中 `rmcs_core`/`rmcs_executor`/`rmcs_utility` 为 `CMAKE_CXX_STANDARD 23`，`rmcs_description`/`rmcs_msgs`/`fast_tf` 为 `-std=c++20`，`rmcs_bringup` 未设；v2 全仓 c++20 | 统一口径（建议全仓 23 或明确分层），并修订 `AGENTS.md` |
| CC2 | **librmcs 集成方式** | 05/06/07 | v2 = 内嵌子模块 + libusb；v3 = FetchContent 外部 SDK `librmcs-sdk` 3.3.0-alpha.7（JGROBO fork） | 以 v3 为准；确认 SDK 版本锁定与网络可用性 |
| CC3 | **`ChassisMode` 枚举不兼容** | 04/08/14/18 | v2 `AUTO/SPIN/STEP_DOWN/LAUNCH_RAMP/AUTO_PILOT`；v3 `AUTO/SPIN_SLOW/SPIN_FAST/STEP_DOWN/LAUNCH_RAMP/ALIGNMENT/ALIGNMENT_POWERED/CLIMB`，删 `AUTO_PILOT` | 定映射表；移植 v2 底盘代码时同步改枚举 |
| CC4 | **`rmcs_msgs/operate_mode.hpp` 缺失** | 04/08/14/18 | v2 有 `OperateMode::MANUAL/ASSIST`；v3 无 | 决定：回迁头文件 / 用新语义替代 |
| CC5 | **`rmcs_msgs/autopilot.hpp` 缺失** | 04/12 | v2 `PilotDiag/NavMode`，被 diagnosis 引用；导航不在范围 | 诊断主题改为不依赖导航（或去掉相关检查） |
| CC6 | **executor `before_pairing` 签名变更** | 03 | v2 `map<string,const type_info&>`；v3 `OutputInfoMap`（用 `output.type.get()`） | v2 覆写点（`value_broadcaster.cpp:22`）已在 v3 适配；移植代码需按 v3 签名 |
| CC7 | **`RingBuffer` API 改名 + `noexcept` 约束** | 16/05/06/07 | `writeable→writable`、`front/back→peek_*`、`*_multi→*_n`；回调须 `noexcept` | 移植代码按 v3 `rmcs_utility::RingBuffer` 改写 |
| CC8 | **`plugins.xml` 注册与归档需成对** | 01/05/06/07/14/18 | v3 已出现 `JOmniWheelController` **重复注册**；v2 `plugins.xml` 残留指向归档文件的 `<class>` | 清理重复注册；新增类必须注册；归档类必须同步移除注册 |
| CC9 | **config 键名 ↔ 代码键名不一致** | 08/18 | `*_limit`(v2键) vs `*_max`(v3读)；`wheel_radius_` vs `wheel_radius`；v3 新增必读 `mass/moment_of_inertia` | 全机型统一键名；P0 用 dual-infantry 作样板 |
| CC10 | **`hikcamera`/`serial` 包缺失但被声明依赖** | 01/02/17 | v3 源码树无 `hikcamera`/`serial` 包，`package.xml` 仍 `depend` | 决策：走 sysroot 提供 / 补包 / 改为可选依赖 |
| CC11 | **两套底盘功率控制器并存** | 08/18 | v3 同时有上游 `ChassisPowerController`（电压制/充电功率/UI）与 `JChassisPowerController`（v2 等价） | 明确各机型用哪套，避免 `chassis_power_controller` 实例名冲突 |
| CC12 | **自瞄子模块实质性分叉** | 17/18 | v2 `2c3122f(dev/25rmul)` vs v3 `8df0266(dev/rk3588npu)`，分叉 23/174，246 文件差异；接口改名 `/gimbal/auto_aim/*`→`/auto_aim/*` | 按 v3 revision 对齐；不整体回退子模块 |

---

## 5. 迁移批次与依赖

> 顺序：**Batch 0（基线/决策）→ 1（地基）→ 2（硬件）→ 3（控制器）→ 4（裁判/UI）→ 5（集成/视觉）→ 6（工程/收口）**。
> 同批次内主题可并行；跨批次严格按依赖。

### Batch 0 — 基线与决策（无代码改动，或仅文档）
- 建立编译/运行基线（§3）。
- 拍板 CC1–CC12 的决策（记录到本文件 §9 与相关主题 §7）。
- **产出**：本人确认 + 基线日志。

### Batch 1 — 运行时与接口地基（阻塞其余全部）
| 主题 | 内容 |
|---|---|
| `03 组件运行时-rmcs_executor` | 确认 v3 executor 能力可承载移植组件（事件接口/统计/跳帧）；必要时补 `components` 缺失报错 |
| `04 消息与接口` | 处理 CC3/CC4/CC5：枚举映射、`operate_mode`/`autopilot` 去留；确认 `atrm_interfaces` 无需迁移（v2 无消费者） |
| `16 通用工具-rmcs_utility` | v3 为超集；仅需按 CC7 改写移植代码 |

### Batch 2 — 硬件栈（机型可"起得来"）
| 主题 | 内容 | 优先级 |
|---|---|---|
| `05 硬件栈-librmcs与设备驱动` | 设备驱动差异清单落地（DjiMotor/DmMotor/LkMotor/supercap/IMU/DR16/VT13 等） | P0 |
| `06 硬件模型-dual_infantry` | **P0**：板卡 serial、pitch CAN 通道、yaw 电机类型、IMU 映射、`/referee/serial`、supercap、GY614、buzzer、`imu_bias` | **P0** |
| `07 硬件模型-其余机型` | dual-sentry/hero/mini-infantry 对齐；`pid-tune-tool` 缺失 | P1 |

### Batch 3 — 控制器
| 主题 | 内容 | 优先级 |
|---|---|---|
| `08 控制器-底盘` | **P0**：`OperateLogic` 语义迁移、`ChassisController` 对齐、J* 校验、CC3/CC9/CC11 | **P0** |
| `09 控制器-云台` | DualYawSolver 参数、SimpleGimbal 丢失项、two_axis 重写、SMC 力矩量纲 | P0 |
| `10 控制器-发射` | 17mm 触发策略、42mm 分叉、HeatController 接入 | P0 |
| `11 控制器-PID与滤波与数学工具` | `PidController::enable` 回归（消除死键）、命名空间迁移 | P1 |
| `12 诊断-diagnosis` | 诊断能力迁移（蜂鸣器受 SDK 限制，CC5） | P1 |

### Batch 4 — 裁判系统与 UI
| 主题 | 内容 | 优先级 |
|---|---|---|
| `13 裁判系统-Status与Command` | 接口重命名/字段变化对齐（`remain_time`→`stage_remain_time` 等） | P1 |
| `14 裁判系统-UI` | **P0（对 dual-infantry）**：新建 `ui::DualInfantry`（+DualSentry/MiniInfantry）、依赖 widget | **P0** |
| `15 TF与描述` | 基本一致；处理 `sentry_description.hpp` 与 `tf_description.hpp` 同名 `Tf` 冲突隐患 | P2 |

### Batch 5 — 集成与视觉
| 主题 | 内容 | 优先级 |
|---|---|---|
| `18 bringup-配置与launch` | **P0**：补齐 `dual-infantry.yaml`（referee/UI/heat/power/diagnosis/auto_aim）、键名统一、`pid-tune-tool` 配置 | **P0** |
| `17 视觉子系统-rmcs_auto_aim_v2` | 接口对齐 `/auto_aim/*`；补 `AutoAimCapturerComponent` 入配置；CC12 | P0（dual-infantry 需要） |

### Batch 6 — 工程链路与收口
| 主题 | 内容 | 优先级 |
|---|---|---|
| `01 仓库布局与包结构` | 布局/包结构记录与必要清理 | P2 |
| `02 工程链路-构建与容器与CI` | CC1/CC10、CI smoke 覆盖、`cross_build.md` 文档纠正、`.clang-format` 统一 | P1 |
| 收口 | 全量 arm64 交叉编译 + 关键模块单测/静态验证 + `clang-format`/`clang-tidy` | — |

---

## 6. 主题 → 批次映射与状态

| 主题 | 批次 | 优先级 | 依赖 | 状态 |
|---|---|---|---|---|
| 01 仓库布局与包结构 | 6 | P2 | — | 待办 |
| 02 工程链路-构建与容器与CI | 6 | P1 | CC1/CC10 | 待办 |
| 03 组件运行时-rmcs_executor | 1 | P0 | — | 待办 |
| 04 消息与接口-rmcs_msgs与interface | 1 | P0 | CC3/CC4/CC5 | 待办 |
| 05 硬件栈-librmcs与设备驱动 | 2 | P0 | 04 | 待办 |
| 06 硬件模型-dual_infantry | 2 | **P0** | 05 | 待办 |
| 07 硬件模型-其余机型 | 2 | P1 | 05 | 待办 |
| 08 控制器-底盘 | 3 | **P0** | 04/05/06 | 待办 |
| 09 控制器-云台 | 3 | P0 | 04/06 | 待办 |
| 10 控制器-发射 | 3 | P0 | 04/06 | 待办 |
| 11 控制器-PID与滤波与数学工具 | 3 | P1 | — | 待办 |
| 12 诊断-diagnosis | 3 | P1 | 05/06 | 待办 |
| 13 裁判系统-Status与Command | 4 | P1 | 04 | 待办 |
| 14 裁判系统-UI | 4 | **P0** | 13/15 | 待办 |
| 15 TF与描述 | 4 | P2 | — | 待办 |
| 16 通用工具-rmcs_utility | 1 | P1 | — | 待办 |
| 17 视觉子系统-rmcs_auto_aim_v2 | 5 | P0 | 18 | 待办 |
| 18 bringup-配置与launch | 5 | **P0** | 06/09/10/14 | 待办 |

---

## 7. 验收（每批次）

1. **编译**：`./.script/build-rmcs-cross --target-arch=arm64 --packages-skip rmcs_auto_aim_v2 --executor sequential` 通过，`rmcs_core` 无新增 warning。
2. **插件**：`nm -C rmcs_ws/install-cross-arm64/lib/librmcs_core.so | grep registerPlugin<...>` 命中新增类。
3. **单测/静态**：关键移植模块（如 J* 控制器、RingBuffer 用法、solver）补少量单测或 `-fsyntax-only` 静态核验。
4. **格式/静态检查**：`clang-format --dry-run -Werror`；尽量过 `clang-tidy`。
5. **离线接口核对**：确认 config 中引用的每个接口都有硬件/控制器的 `register_output` 来源（无悬空 topic）。
6. **实机/仿真冒烟（如有条件）**：`set-robot dual-infantry && launch-rmcs`，无 `respawn` 循环，topic 巡检见各主题 §6。

---

## 8. 风险登记

| 风险 | 影响 | 缓解 |
|---|---|---|
| 实机事实未确认（serial/CAN 通道/电机模式/零点/IMU 映射） | 移植后"能编译不能跑" | Batch 0 必须先确认（§9） |
| 复用旧 `librmcs_core.so` | 验证结果失真 | 每次验证前重新构建/安装 |
| v2 源码格式与 v3 `.clang-format` 不同 | diff 噪声大、review 困难 | 移植后立即 `clang-format`；分批提交 |
| 自瞄子模块分叉大 | 整体回退引入回归 | 按 v3 revision 增量对齐接口，不回退子模块 |
| 两套功率控制器/枚举并存 | 语义混乱 | CC11/CC3 明确选用与映射 |
| 缺 `mass/moment_of_inertia` 等必读参数 | 节点 `respawn` 循环 | CC9 全机型补齐 |

---

## 9. 待用户/实机确认的问题（Batch 0 阻塞）

### 9.1 已确认决策（2026-10-19，用户答复）

| # | 问题 | 决策 |
|---|---|---|
| CC1 | C++ 标准 | **维持现状分层**（核心包 C++23、其余 C++20；仅在移植处适配，不强行统一） |
| CC4/CC5 | `operate_mode`/`autopilot` 语义 | **完全放弃操作辅助**：不迁移 `OperateLogic`、不回迁 `operate_mode.hpp`；相关 config 键（`operate_logic.*`、`/chassis/operate_mode*`、`/chassis/boost` 等）一并清理。`diagnosis` 中对 autopilot 的依赖按 v3 现状去掉。 |
| CC10 | `hikcamera`/`serial` 依赖 | **目标环境 / sysroot 已提供，无需处理**（仅确认构建可通过，不改依赖声明） |
| CC11 | dual-infantry 功率控制 | **`JChassisPowerController`（v2 等价）**；需配合接回 supercap |
| 实机事实 | 主题 06 未核实项 | **先按 v2 值为准迁移**（pitch 回 CAN2、IMU 用 v2 恒等映射、yaw 用电流环 `GM6020`、回迁 `imu_bias_x/y/z`），每处加 TODO 注释，实机再标定 |
| 主题17 | 自瞄接入 | **必须接入**；相机内参用 **v3 的 `AUTOAIM_CONFIG` 机制**（不恢复 `RMCS_ROBOT_TYPE` 分支） |
| 范围 | 阶段二改动范围 | **只做 dual-infantry（P0）**；其余机型 / `pid-tune-tool` **仅出文档**，不改代码 |
| 粒度 | 执行与提交 | **按主题分批，每批一次 arm64 交叉编译验证，不中途 review**（自主推进） |

**由上述决策推导的执行约束**：
- 不再新增 `rmcs_msgs/operate_mode.hpp`；`04` 主题中该项标为「放弃迁移」。
- `08` 主题的 `OperateLogic` 相关 checklist **跳过**；`18` 中 `operate_logic` 参数块**删除**。
- 除 dual-infantry 外的机型：仅**修正会阻断 dual-infantry 的共享项**（如 `ChassisMode` 枚举、`RingBuffer` 用法）时顺带处理；不做机型专属移植。
- 移植代码若使用 C++23 特性，仅放在 C++23 的包内（`rmcs_core`/`rmcs_executor`/`rmcs_utility`）。

### 9.2 仍待实机确认（不阻塞编码，先按 v2 值 + TODO）
两板真实 serial、pitch 物理 CAN 通道、yaw 电机控制模式、TopBoard IMU 坐标映射、三组零点是否同一台车、`imu_bias` 是否仍需。

> 上述 8 项已于 2026-10-19 由用户答复，见 §9.1；原「待确认」列表已由 9.1/9.2 取代。

### 9.3 已确认决策（2026-10-19，第三轮：阶段三）

| # | 问题 | 决策 |
|---|---|---|
| 方向 | 下一阶段主方向 | **P1 其余机型全链路**（dual-sentry / mini-infantry / mec-hero）：07 硬件 + 14 UI + 18 config 一并推进 |
| 范围 | 机型顺序 | **三机型一次做完**；选定方向的机型**允许修改代码** |
| 单测 | 关键模块单测/静态 | **不新建 gtest 基建**；验收以 arm64 交叉编译 + `clang-tidy`/`-fsyntax-only` 静态核验为准 |
| 工具 | `pid-tune-tool` | **方案 B：弃用** `PidTuneTool`，改用 `identification::{StaticTorqueTestController,SweptFrequencyController}` + `debug::ValueCollector`，并补一份可运行 config |
| 提交 | git | **分主题提交到本地仓库**（docs-mig 与代码分开，每主题/每批一 commit） |

**由阶段三决策推导的执行约束**：
- 阶段三改动范围 = `dual-sentry` / `mini-infantry` / `mec-hero` 三机型（硬件/config/UI），以及 `pid-tune-tool` 相关的弃用与替代 config。
- 三机型同样遵循「先按 v2 值迁移 + TODO」处理实机事实（serial/IMU 映射/电机模式/零点）。
- 不引入新的测试框架；不修改 `rmcs_executor`/`rmcs_utility` 公共接口。
- 每完成一个主题即 `git commit`（本地仓库，不 push）。

---

## 10. 进度追踪

- 见 `README.md` 的「进度表」；每完成一个主题，更新其 `topics/*.md` 顶部状态与 `README.md`。
