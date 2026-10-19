# 迭代后的任务提示词（v1）

> 本文件是经用户问答迭代后确定的「任务定义」。后续所有主题差异文件与迁移计划均以本文件为准。
> 如与本文件不符，以本文件 + `plans/00-master-plan.md` 为准。

## 1. 任务目标

对 **jRMCS（v3，新版架构）** 与 **reATRM（v2，旧版架构）** 的工作链路进行**全量比对**，
找出两者在架构、功能、实现、调参上的**全部差异**，产出一份**逐主题的差异档案 + 可执行的迁移计划**，
最终**以 v3 架构为骨架，把 v2 中经实车验证可用的车型功能 / 控制器 / 调参 / 工具移植进 v3（扬长避短）**。

- **jRMCS（目标，可写）**：`/workspaces/RMCS`
  - 架构新版 v3；车型功能 / 代码**不全**。
  - 当前分支：`dev/jgrobo-ize`（已有一批迁移提交）。
  - 工作区：`rmcs_ws/src/{rmcs_executor, rmcs_core, rmcs_msgs, rmcs_description, rmcs_utility, rmcs_bringup, fast_tf, rmcs_auto_aim_v2}`。
- **reATRM（参照，只读）**：`/workspaces/reATRM`
  - 架构旧版 v2；老，但**具体车型能够工作**。
  - 分支：`allinone`；工作区：`src/{bringup, core, interface, tasks}`。
- 两者源自同一上游项目（`Alliance-Algorithm/RMCS`），**可从 git 记录获得理解**（jRMCS 的 `dev/jgrobo-ize` 已含迁移提交，可对照）。

## 2. 已确认的决策（用户答复）

| 项 | 决策 |
|---|---|
| **比对范围** | ① 运行时全链路（executor、接口/消息、设备驱动、硬件模型、控制器、裁判系统/UI、TF 描述）；② 工程链路（`.script`、CMake、交叉编译工具链、Dockerfile、CI、devcontainer、部署/同步）；③ 视觉子系统 `rmcs_auto_aim_v2`。 |
| **不在范围** | 导航 / SLAM / `autopilot_client` 等外设子系统（**不纳入本次全量比对**，可在文档中作为“已知未覆盖项”留档）。 |
| **迁移方向** | **以 v3 架构为骨架**，把 v2 中可用的车型功能 / 调参 / 控制器**移植进 v3**；尽量少改 v3 架构，不改上游骨架。 |
| **验收标准** | ① `build-rmcs-cross --target-arch arm64` 全量通过（可 `--packages-skip rmcs_auto_aim_v2`）；② 对**关键移植模块**补充少量单元测试 / 静态验证；③ `clang-format` / `clang-tidy` 通过。 |
| **机型范围** | **dual-infantry 为 P0 重点**（用户单选）。其余机型（dual-sentry / mini-infantry / mec-hero / pid-tune-tool）作为次级，仍记录差异但优先级低于 dual-infantry。 |
| **文档组织** | `docs-mig/`：`README.md` 索引 + `prompts/`（迭代后的提示词）+ `plans/`（总计划）+ `topics/`（每主题一个差异文件）；**全部中文**；**纳入 git**。 |
| **执行边界** | **先把全部 `docs-mig` 文档写完，等用户确认后再动代码**。 |

## 3. 交付物（docs-mig 结构）

```
docs-mig/
├── README.md                     # 索引：主题清单、状态、进度、决策速览
├── prompts/
│   └── 00-prompt-v1.md           # 本文件：迭代后的任务提示词
├── plans/
│   ├── 00-master-plan.md         # 总迁移计划：工作流、依赖、批次、验收
│   └── 01-diff-matrix.md         # 差异总览矩阵与迁移依赖排序
└── topics/
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

## 4. 工作流（两阶段）

**阶段一：建档（本阶段，先做完）**
1. 逐主题比对 jRMCS 与 reATRM，产出 `topics/*.md`。
2. 每个主题文件必须做到**可追溯**：给出两侧**确切文件路径**、**类名/函数名/符号名**、**关键行或代码片段**、以及**差异点**。
3. 每个主题文件末尾给出**可执行的迁移步骤（checklist）**，并标注**依赖**与**验收方式**。
4. 汇总 `plans/01-diff-matrix.md`：主题 × 差异等级 × 影响机型 × 迁移依赖 × 批次。
5. 更新 `README.md` 索引与进度。

**阶段二：执行（待用户确认后）**
1. 按 `plans/00-master-plan.md` 的批次与依赖，逐个 `topics/*.md` 执行迁移。
2. 每个主题执行后：`clang-format` → 交叉编译 arm64 验证 →（关键模块）补单测 / 静态验证。
3. 每完成一个主题，更新该文件的状态与 `README.md` 进度表。

## 5. 主题文件统一模板

每个 `topics/*.md` 使用以下结构：

```markdown
# <主题名>

- 状态：待办 / 进行中 / 已完成
- 优先级：P0 / P1 / P2
- 影响机型：
- 依赖主题：
- 验收方式：

## 1. v2（reATRM）现状
（文件路径、类/函数、行为、参数）

## 2. v3（jRMCS）现状
（文件路径、类/函数、行为、参数）

## 3. 差异清单
（逐条：差异点 / v2 做法 / v3 做法 / 影响 / 风险）

## 4. 迁移方案（以 v3 为骨架）
（具体到文件与符号：新增/修改/移植/归档）

## 5. 执行步骤 checklist
- [ ] ...

## 6. 验收与验证
（编译、单测、静态检查、实机/离线说明）

## 7. 备注与未决问题
```

## 6. 约束与红线

- **reATRM 只读**：不得修改 `/workspaces/reATRM` 下任何文件。
- 阶段一**不得修改 jRMCS 代码**（只写 `docs-mig/`）；阶段二经用户确认后方可改代码。
- 交叉编译验证日志写入可写目录（如 `/tmp/opencode/` 或 `rmcs_ws/log-cross-arm64/`）。
- 保持与周边代码风格一致（C++20、命名约定见 `AGENTS.md`）。
- 移植 v2 代码时注意 **C++ 标准差异**（v2 `c++20` vs v3 `CMAKE_CXX_STANDARD 23`）与 **librmcs 集成方式差异**（v2 内嵌 `librmcs` 子模块 + libusb vs v3 FetchContent 外部 SDK）。

## 7. 决策记录（第二轮问答，2026-10-19）

| # | 问题 | 决策 |
|---|---|---|
| CC1 | C++ 标准 | 维持现状分层（核心包 23 / 其余 20），仅移植处适配 |
| CC4/CC5 | operate_mode / autopilot | **完全放弃操作辅助**；不回迁，清理相关 config 键 |
| CC10 | hikcamera / serial 依赖 | 目标环境/sysroot 已提供，无需处理 |
| CC11 | dual-infantry 功率控制 | **`JChassisPowerController`**（v2 等价），需接回 supercap |
| 实机事实 | 06 未核实项 | 先按 v2 值迁移（pitch→CAN2、IMU 恒等映射、yaw `GM6020`、回迁 `imu_bias`），加 TODO，实机再标定 |
| 主题17 | 自瞄 | 必须接入；相机内参用 v3 `AUTOAIM_CONFIG` 机制 |
| 范围 | 改动范围 | **只做 dual-infantry**；其余机型 / pid-tune-tool 仅出文档 |
| 粒度 | 执行/提交 | 按主题分批，每批一次 arm64 交叉编译，不中途 review |

> 由此确立：`04` 的 operate_mode 项标为「放弃迁移」；`08` 的 `OperateLogic` checklist 跳过；`18` 删除 `operate_logic` 参数块；C++23 特性仅用在 23 的包内。

## 8. 决策记录（第三轮问答，2026-10-19）

| # | 问题 | 决策 |
|---|---|---|
| 方向 | 下一阶段 | **P1 其余机型全链路**：dual-sentry / mini-infantry / mec-hero |
| 范围 | 机型顺序 | **三机型一次做完**（允许改代码） |
| 单测 | 关键模块验证 | **不新建 gtest 基建**；验收 = arm64 交叉编译 + `clang-tidy`/`-fsyntax-only` |
| 工具 | `pid-tune-tool` | **方案 B：弃用**，改用 `identification::*` + `debug::ValueCollector` + 可运行 config |
| 提交 | git | **分主题提交到本地仓库**（docs-mig 与代码分开） |
