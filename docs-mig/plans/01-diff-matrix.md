# 01 差异总览矩阵与迁移依赖

> 本文件汇总 `topics/*.md` 的结论，供执行阶段排序与验收。
> 详细证据见各主题文件；本文件只做索引、分级与依赖。

---

## 1. 主题 × 差异等级 × 批次

差异等级：**L1 = 架构级（影响能否编译/加载）**；**L2 = 功能级（能跑但缺功能/行为不同）**；**L3 = 参数/风格级**。

| 主题 | 差异规模 | 最高等级 | 主要影响 | 批次 | 状态 |
|---|---|---|---|---|---|
| 01 仓库布局与包结构 | 12 包→8 包；4→2 子模块 | L1 | 全局（路径/包名/子模块） | 6 | 待办 |
| 02 工程链路-构建与容器与CI | v3 独有交叉编译/CI/同步；28 条差异 | L1 | 全局（C++ 标准、librmcs、依赖） | 6 | 待办 |
| 03 组件运行时-rmcs_executor | 大改（事件/统计/跳帧/before_pairing） | L1 | 全局 | 1 | 待办 |
| 04 消息与接口 | +7 头 / −2 头 / 改 2 头 | L1 | 全机型 | 1 | 待办 |
| 05 硬件栈-librmcs与设备驱动 | 架构级重写；6 头增/2 头删/9 头重写 | L1 | 全机型 | 2 | 待办 |
| 06 硬件模型-dual_infantry | 21 条硬件差异 + config 差异 | L1 | **dual-infantry** | 2 | 待办 |
| 07 硬件模型-其余机型 | 三机型自洽；pid_tune_tool 缺失 | L2 | sentry/hero/mini | 2 | 待办 |
| 08 控制器-底盘 | J* 已等价迁移；OperateLogic 全缺；ChassisController 分叉 | L1 | 全机型 | 3 | 待办 |
| 09 控制器-云台 | controller 等价；solver/simple/two_axis 有实质差异 | L2 | 全机型 | 3 | 待办 |
| 10 控制器-发射 | 17/42mm 分叉；heat 未接 | L2 | 全机型 | 3 | 待办 |
| 11 控制器-PID与滤波与数学工具 | `PidController::enable` 被删（10 处死键） | L2 | 多机型 | 3 | 待办 |
| 12 诊断-diagnosis | v2 蜂鸣器诊断 → v3 几乎为空、无消费者 | L2 | 全机型 | 3 | 待办 |
| 13 裁判系统-Status与Command | 接口重命名/字段类型变化；+SentryDecision | L2 | 全机型 | 4 | 待办 |
| 14 裁判系统-UI | v3 无 dual_infantry/dual_sentry/mini_infantry UI | L1 | 全机型 | 4 | 待办 |
| 15 TF与描述 | 基本一致；+sentry_description（同名 `Tf` 冲突隐患） | L3 | sentry | 4 | 待办 |
| 16 通用工具-rmcs_utility | v3 超集；`ring_buffer.hpp` 冲突 | L2 | 全机型 | 1 | 待办 |
| 17 视觉子系统-rmcs_auto_aim_v2 | 子模块实质分叉；接口全改名；采集组件缺席 | L1 | dual-sentry/infantry 等 | 5 | 待办 |
| 18 bringup-配置与launch | dual-infantry 23→13 组件；键名不一致 | L1 | **dual-infantry** | 5 | 待办 |

---

## 2. 依赖图（执行顺序）

```
Batch0 基线/决策(CC1..CC12)
   │
   ├─▶ Batch1  03 executor ─┐
   │            04 msgs ─────┤ (L1 地基)
   │            16 utility ──┘
   │                 │
   ├─▶ Batch2  05 设备驱动 ──▶ 06 dual_infantry(P0) ──▶ 07 其余机型
   │                 │
   ├─▶ Batch3  08 底盘(P0) · 09 云台 · 10 发射 · 11 PID · 12 诊断
   │                 │
   ├─▶ Batch4  13 Status/Command ──▶ 14 UI(P0) ; 15 TF
   │                 │
   ├─▶ Batch5  18 bringup(P0) ──▶ 17 auto_aim
   │                 │
   └─▶ Batch6  01 布局 · 02 工程链路 ──▶ 收口验证
```

---

## 3. 主题间耦合（必须协同执行）

| 耦合 | 主题 | 说明 |
|---|---|---|
| 枚举 `ChassisMode` / `operate_mode` | 04 ↔ 08 ↔ 14 ↔ 18 | 一处改枚举，配置与 UI 同步 |
| 底盘参数键名 | 08 ↔ 18 ↔ 07 | `*_max` / `mass` / `moment_of_inertia` 代码与配置成对 |
| 设备接口名（`velocity_filtered` 等） | 05/06 ↔ 08/10/18 | 硬件不再注册 → PID 的 `measurement` 悬空 |
| 裁判串口 `/referee/serial` | 06 ↔ 13/14/18 | 硬件丢失 → 裁判链无输入 |
| 自瞄接口 `/auto_aim/*` | 17 ↔ 04 ↔ 08/10/14 ↔ 18 | 组件、控制器、UI、配置全链改名 |
| 插件注册 ↔ 配置 ↔ 归档 | 01/05/06/07/14 ↔ 18 | 新增类须注册 + 入配置；归档须移除注册 |
| 功率控制器二选一 | 08 ↔ 18 ↔ 06 | `JChassisPowerController` vs 上游，且依赖 supercap |

---

## 4. 关键差异速览（按批次）

### Batch 1 地基
- v3 executor 用 `OutputInfoMap`/事件接口/统计/跳帧；`before_pairing` 签名变更（CC6）。
- `rmcs_msgs`：`ChassisMode` 重排删 `AUTO_PILOT`（CC3）；删 `operate_mode.hpp`（CC4）、`autopilot.hpp`（CC5）；`RobotId::color()` 未知时返回变 UNKNOWN。
- `rmcs_utility` 为超集；`RingBuffer` API 改名 + `noexcept`（CC7）。

### Batch 2 硬件
- librmcs：子模块→FetchContent SDK；CAN 组包 `uint64_t`→`CanPacket8`；驱动下沉到 `rmcs_core/hardware/device`（CC2）。
- dual-infantry 缺口：`/referee/serial`、supercap、buzzer、GY614 温度、`imu_bias`；行为改变：pitch CAN 通道、yaw 电机类型、IMU 映射矩阵、TF 100Hz（CC9/主题06 §3）。
- 其余机型自洽；`pid_tune_tool` 全缺。

### Batch 3 控制器
- 底盘：v2 扭矩分配/自旋平移**已完整搬入** `JOmniWheelController`；`JChassisPowerController` 等价 v2；但 `OperateLogic` 全缺、`ChassisController` 深度分叉（autopilot/vmax/ASSIST 丢失）。
- 云台/发射/PID/诊断见各主题（`PidController::enable` 死键、HeatController 未接、诊断近乎为空）。

### Batch 4 裁判/UI/TF
- v3 UI 无机型 UI；现役 config 未接 `referee/app::ui` → 当前无 UI 输出。
- Status 接口重命名与类型变化；`/referee/robot/hurt`、`/referee/game/center_area` 在 v3 被移除。
- TF/描述基本一致，仅新增 `sentry_description.hpp`（同名 `Tf` 隐患）。

### Batch 5 集成/视觉
- 视觉子模块 v2(dev/25rmul) vs v3(dev/rk3588npu) 实质分叉；接口 `/gimbal/auto_aim/*`→`/auto_aim/*`；采集组件未入任何机型配置。
- `dual-infantry.yaml` 是最缺配置的文件（23→13 组件）。

### Batch 6 工程
- C++ 标准分层（CC1）；`hikcamera`/`serial` 依赖（CC10）；CI smoke 只到 `rmcs_executor`；`cross_build.md` 与实现冲突；`.clang-format` 两仓不一致。

---

## 5. 未决问题去向

执行时需先解决的未决问题见 `plans/00-master-plan.md §9`（CC1–CC12 + 实机事实 + 范围/粒度）。
