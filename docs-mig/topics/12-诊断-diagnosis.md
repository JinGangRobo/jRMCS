# 诊断-diagnosis

- 状态：待办
- 优先级：P1（v2 的蜂鸣器与云台离线提示已整体缺失；dual-infantry 的 P0 影响见 09-D8）
- 影响机型：dual-infantry（v3 **未启用** diagnosis）、dual-sentry、mini-infantry、mec-hero（v3 已启用但功能缩水）
- 依赖主题：05（硬件栈/librmcs 与设备驱动，蜂鸣器必须由硬件侧提供）、06（dual_infantry 硬件模型，提供 `/gimbal/*/alive`、`/chassis/*/alive`、`/chassis/supercap/enabled`）、04（`rmcs_msgs`，v2 的 `PilotDiag`）、09（云台离线）
- 验收方式：交叉编译 arm64 通过；`clang-format` 通过；静态核对 `/diagnosis/status`（或 `/buzzer/score`）的生产者/消费者；实车验证上电蜂鸣提示、电机掉线提示、超电掉线提示

## 0. 本次核对的结论摘要

1. `controller/diagnosis.cpp` **两版本质不同**：v2 输出 `/buzzer/score`（`std::optional<librmcs::device::Buzzer::BuzzerScore>`）驱动**蜂鸣器**；v3 只输出 `/diagnosis/status`（`uint8_t`），**无蜂鸣器**。任务提示中「v3 迁移提交提到 no buzzer」**属实**——v3 仓库内**完全没有** `buzzer` 相关文件与符号（`grep -rn "buzzer\|Buzzer" rmcs_ws/src` 无命中）。
2. v2 的诊断项包含 **电机离线 / 超电离线 / 自瞄就绪 / 导航（autopilot）6 种状态**；v3 只剩 **电机离线 / 超电离线** 2 项。
3. v2 有硬件层 `hardware/device/buzzer.hpp`（`class Buzzer : librmcs::device::Buzzer`）与内嵌 `librmcs` 的 `librmcs/device/buzzer.hpp`；v3 的 `hardware/device/` 目录**没有** buzzer，改用外部 SDK（`FetchContent` 拉取 `libjrmcs` 的 `librmcs-sdk-src-3.3.0-alpha.7`），**本仓库内无法确认该 SDK 是否提供蜂鸣器 API**（标注「未核实」）。
4. v3 **未在 `dual-infantry.yaml` 启用 `diagnosis`**（v2 启用了），且 `dual-infantry` 也没有 `ChassisPowerController` → `/chassis/supercap/enabled` 缺席。
5. 电机的 `/alive` 输入在 v2/v3 都是**必选**（`register_input(motor + "/alive", *motor_alive_input)`），即诊断组件仍**要求**被检查组件加载；v2 提交 `2ebb81e` 只把超电与自瞄输入改成了可选。

---

## 1. v2（reATRM）现状

文件：`/workspaces/reATRM/src/core/utils/system_ctrl/rmcs_core/src/controller/diagnosis.cpp`（162 行）

### 1.1 枚举与蜂鸣谱

```cpp
using Buzzer = librmcs::device::Buzzer;
using Tone   = librmcs::device::Buzzer::Tone;

enum class DiagnosisMsg : uint8_t {   // larger value means higher priority
    ALL_READY, CTR_MOTOR_OFFLINE, AIM_READY, NAV_SLAMING,
    CTR_SUPERCAP_OFFLINE, NAV_WARNING, NAV_RELOCATION_FAILED, NAV_FAILURE, NAV_OFFLINE
};
constexpr std::array<std::pair<DiagnosisMsg, Buzzer::BuzzerScore>, 8> DiagnosisScores = { ... };
constexpr Buzzer::BuzzerScore getScoreFromDiagMsg(DiagnosisMsg msg) { ... }
```
（L15-48；`DiagnosisScores` 给每个状态定义了 3 音 + 第 4 字段的蜂鸣谱）

### 1.2 参数与接口

- 参数：`motors`（数组）、`enable_motor_check`、`enable_supercap_check`、`enable_aim_check`、`enable_nav_check`（L59-63）
- 输入：
  - `motor + "/alive"`（**必选**，L67）
  - `/chassis/supercap/enabled`（可选，L70）
  - `/gimbal/auto_aim/available`（可选，L71）
  - `/autopilot/diagnosis`（`rmcs_msgs::PilotDiag`，可选，L72）
- 输出：`/buzzer/score`（`OutputInterface<std::optional<Buzzer::BuzzerScore>>`，L74）

### 1.3 update 逻辑（L77-136）

- 电机：`if (enable_motor_check_) for (...) if (!ready() || !**input) errors_.push_back(CTR_MOTOR_OFFLINE);`
- 超电：`if (enable_supercap_check_) if (!ready() || !*) errors_.push_back(CTR_SUPERCAP_OFFLINE);`
- 自瞄：`if (enable_aim_check_) if (aim_ready_input_.ready() && *aim_ready_input_) errors_.push_back(AIM_READY);`（**就绪也上报**，用于「就绪提示音」）
- 导航：`if (enable_nav_check_) switch (*pilot_diag_input_)` → `NAV_WARNING / NAV_RELOCATION_FAILED / NAV_FAILURE / NAV_OFFLINE / NAV_SLAMING`
- 汇总：取优先级最高（枚举值最大）者映射为蜂鸣谱写入 `/buzzer/score`；无错误时写 `std::nullopt`

### 1.4 硬件侧蜂鸣器

`/workspaces/reATRM/src/core/utils/system_ctrl/rmcs_core/src/hardware/device/buzzer.hpp`
```cpp
class Buzzer : librmcs::device::Buzzer {
    Buzzer(rmcs_executor::Component& command_component) {
        command_component.register_input("/buzzer/score", buzzer_score_input_, false);  // f8a9cd2 改为可选
        set_score({Tone::LOW, Tone::MEDIUM, Tone::HIGH, 0}, Mode::ONCE);
    }
    void update_status() {
        if (buzzer_score_input_.ready()) {
            if (buzzer_score_input_->has_value())
                set_score(buzzer_score_input_->value(), Mode::CONTINUOUS);
            else
                set_score({Tone::OFF, Tone::OFF, Tone::OFF, 0}, Mode::CONTINUOUS);
            librmcs::device::Buzzer::update_status();
        }
    }
    uint8_t generate_command() { return librmcs::device::Buzzer::generate_command(); }
};
```
- 内嵌 SDK 侧：`.../rmcs_core/librmcs/librmcs/device/buzzer.hpp`（`struct BuzzerScore`、`PACKED_STRUCT(BuzzerScoreMsg{...})`、`Mode{ONCE,CONTINUOUS}`）

### 1.5 相关提交（`git -C /workspaces/reATRM log`）

| 提交 | 说明 |
|---|---|
| `82948bc` 添加蜂鸣器诊断提示 | 新增 `controller/diagnosis.cpp`（154 行）、`hardware/device/buzzer.hpp`，改 4 个 config、`plugins.xml`、硬件模型 |
| `2ebb81e` 使诊断组件不强制要求被检查组件加载 | 把 `/chassis/supercap/enabled`、`/gimbal/auto_aim/available` 改为可选（`register_input(..., false)`） |
| `f8a9cd2` 解除自瞄或诊断等组件间多余的耦合 | `buzzer.hpp` 的 `/buzzer/score` 改为可选；同步改各 UI |
| `a2bef86` / `5165cd9` / `a4ebfd4` | 导航诊断相关修复（`switch` 异常 return） |

### 1.6 v2 各机型配置

4 个机型（`dual-infantry.yaml:217`、`dual-sentry.yaml:201`、`mec-hero.yaml:192`、`mini-infantry.yaml:187`）均启用，`motors` 列表覆盖云台 + 摩擦轮 + 拨弹 + 四个底盘轮；`enable_motor_check: true`、`enable_supercap_check: true`、`enable_aim_check: true`、`enable_nav_check: false`。

---

## 2. v3（jRMCS）现状

文件：`/workspaces/RMCS/rmcs_ws/src/rmcs_core/src/controller/diagnosis.cpp`（85 行）

### 2.1 枚举（仅 3 项）

```cpp
enum class DiagnosisMsg : uint8_t { // larger value means higher priority
    ALL_READY = 0,
    CTR_MOTOR_OFFLINE = 1,
    CTR_SUPERCAP_OFFLINE = 2,
};
```
（L13-17）

### 2.2 参数与接口

- 参数：`motors`、`enable_motor_check`、`enable_supercap_check`（L28-30；**无** `enable_aim_check` / `enable_nav_check`）
- 输入：`motor + "/alive"`（**必选**，L34）、`/chassis/supercap/enabled`（可选，L37）
- 输出：`/diagnosis/status`（`uint8_t`，默认 `ALL_READY`，L39-40）

### 2.3 update 逻辑（L43-66）

- 电机 / 超电判定与 v2 逐行相同
- 汇总：`highest = max_element(...)`，`*status_output_ = errors_.empty() ? ALL_READY : *highest;`（L60-65）
- **无蜂鸣器、无自瞄、无导航**

### 2.4 硬件侧

`rmcs_ws/src/rmcs_core/src/hardware/device/` 下无 buzzer；`supercap.hpp` 输出 `/chassis/supercap/enabled`，`j_supercap.hpp` 亦输出并额外输出 `/chassis/supercap/energy_percentage`。

### 2.5 v3 各机型配置

| 机型 | 启用 diagnosis | 参数 |
|---|---|---|
| `mec-hero.yaml:36` / `:190` | 是 | `motors` + `enable_motor_check: true` |
| `mini-infantry.yaml:32` / `:186` | 是 | `motors` + `enable_motor_check: true` + `enable_supercap_check: true` |
| `dual-sentry.yaml:32` / `:191` | 是 | `motors` + `enable_motor_check: true` |
| `dual-infantry.yaml` | **否** | — |

（v3 config 中**没有** `enable_aim_check`/`enable_nav_check` 残留键。）

---

## 3. 差异清单

### D1 输出接口与类型不同（P1）
- **v2**：`OutputInterface<std::optional<Buzzer::BuzzerScore>>` → `/buzzer/score`；消费者为 `hardware/device/buzzer.hpp`
- **v3**：`OutputInterface<uint8_t>` → `/diagnosis/status`；`grep -rn "diagnosis/status" rmcs_ws/src` **无消费者**（当前无人订阅）
- **影响**：v3 的 `diagnosis` 现在只是「算出来没人用」；v2 的 `/buzzer/score` 有真实消费者
- **风险**：中（v3 诊断实际未生效）

### D2 蜂鸣器能力整体缺失（P1）
- **v2**：`controller/diagnosis.cpp` + `hardware/device/buzzer.hpp` + 内嵌 `librmcs/device/buzzer.hpp`（`BuzzerScore`/`Mode`/`generate_command`）
- **v3**：仓库内**无任何 `Buzzer` 符号**；改用外部 SDK（`CMakeLists.txt:17-25` `FetchContent` `librmcs-sdk-src-3.3.0-alpha.7`）
- **已核实**：该 SDK 已在本地展开到 `rmcs_ws/build-cross-arm64/rmcs_core/_deps/librmcs-src/`，其 `CMakeLists.txt` 中 `LIBRMCS_PROJECT_VERSION = 3.3.0-alpha.7`；`core/include/librmcs/` 下只有 `data/`（`datas.hpp`）与 `spec/{uart,can,gpio}.hpp` + 3 个板卡规格（`rmcs_board_lite`/`c_board`/`rmcs_board_pro`），**共 38 个头文件，`grep -ril "buzzer\|tone"` 无命中**，也**没有 v2 那样的 `librmcs/device/` 目录** → **该 SDK 不提供蜂鸣器 API**
- **影响**：v3 当前**无法**使用 v2 的蜂鸣提示（上电提示音、电机掉线提示、超电掉线提示、自瞄就绪提示）；恢复需先更换/升级 SDK
- **风险**：中

### D3 诊断项缩水（P1）
- **v2**：9 项（含 `AIM_READY`、`NAV_SLAMING/WARNING/RELOCATION_FAILED/FAILURE/OFFLINE`）
- **v3**：3 项
- **影响**：v2 的自瞄就绪提示与导航状态提示消失。注意：**v3 的导航接口已换成 `/rmcs_navigation/*`**（见主题 08 D4），v2 的 `/autopilot/diagnosis`（`rmcs_msgs::PilotDiag`）在 v3 不存在——恢复导航诊断需重新定义枚举，**不是简单回带**
- **风险**：中

### D4 `enable_*` 参数集合不同（P2）
- v2：4 个开关；v3：2 个
- v3 config 已清理，无残留死键 → 无配置侧问题
- **风险**：低

### D5 电机 `/alive` 仍为必选（P2）
- v2 L67 / v3 L34 均为 `register_input(motor + "/alive", *motor_alive_input)`（无 `false`）→ 诊断组件**要求**被检查电机组件存在（否则输入无生产者）
- v2 提交 `2ebb81e` 的标题（「不强制要求被检查组件加载」）**只覆盖了超电/自瞄**，并未覆盖 `motors`
- **影响**：若启用 diagnosis 而某电机未加载，可能报输入缺失
- **风险**：低-中

### D6 v3 `dual-infantry` 未启用 diagnosis（P0，与 09-D8 相关）
- **v2**：`dual-infantry.yaml:34` 启用，`motors` 覆盖全部云台/摩擦轮/拨弹/底盘电机
- **v3**：`dual-infantry.yaml` **无** `diagnosis` 组件 → 云台/底盘**完全没有离线检测**
- **影响**：与 09-D8、10-D4 叠加（`SimpleGimbalController` 无 `depond_motors`、`FrictionWheelController` 无 `/alive` 门控、`ChassisController` 保留了 `wheel_alive_protecter_`）→ 除底盘轮外，其它电机离线无任何保护
- **风险**：**高**

---

## 4. 迁移方案（以 v3 为骨架）

1. **T12-1（P0）** 给 v3 `dual-infantry.yaml` 启用 `diagnosis`
   - 增加 `rmcs_core::controller::diagnosis::Diagnosis -> diagnosis` 与 `diagnosis:` 参数块（`motors` 列表照抄 v2 `dual-infantry.yaml:219-229`；`enable_motor_check: true`；超电相关先置 false，因为该机型无超电）
   - 前提：`motors` 中的每个电机都能提供 `/alive`（依赖 06）
2. **T12-2（P1）** 明确 `/diagnosis/status` 的消费方
   - 方案 A：让 UI（`referee/app/ui/*`）读取 `/diagnosis/status` 并显示（v3 已有 `deformable_infantry_ui` 读 `/chassis/supercap/enabled` 等）
   - 方案 B：接一个「蜂鸣器/指示灯组件」，把 `uint8_t` 映射为提示音/灯效（D2 的替代）
3. **T12-3（P1）** 蜂鸣器能力核实与恢复
   - **已核实**：当前选用的 SDK（`librmcs` `3.3.0-alpha.7`，本地展开于 `build-cross-arm64/rmcs_core/_deps/librmcs-src/`）**不含任何蜂鸣器接口**（无 `device/` 目录，无 `buzzer`/`tone` 符号）
   - 因此恢复路径只有两条：
     - (a) 升级/替换 SDK 到带 `device/buzzer.hpp` 的版本（需确认 `libjrmcs` 是否有该版本）
     - (b) 绕过 SDK，用下位机板卡的通用 IO/CAN 命令自行实现提示音（若板卡固件支持）
   - 若两者都不可行：在本文档记录「v3 无法使用蜂鸣提示」，改为 LED/UI 提示（配合 T12-2 方案 B）
4. **T12-4（P1）** 恢复自瞄就绪提示（`AIM_READY`）
   - v2 语义「`gimbal_takeover_ == true` 即 AIM_READY」；v3 对应 `/auto_aim/should_control`（主题 09 D4）
   - 由于输出已改为 `uint8_t` 状态码，需扩展枚举（例如 `AIM_READY = 3`）或在 UI 侧另行判断
5. **T12-5（P2）** 导航诊断
   - v3 无 `/autopilot/diagnosis`；如需，改为消费 `/rmcs_navigation/*` 并自定义枚举，**不要**直接回带 `rmcs_msgs::PilotDiag`
6. **T12-6（P2）** 让 `motors` 的 `/alive` 变为可选（`register_input(..., false)`）以贴合 v2 提交 `2ebb81e` 的意图；或在文档中说明「motors 必选」是有意为之

---

## 5. 执行步骤 checklist

- [ ] 核实外部 SDK 是否提供蜂鸣器 API（决定 T12-3 路线）
- [ ] `dual-infantry.yaml` 启用 `diagnosis`（motors 列表 + enable_motor_check）
- [ ] 确认 `/diagnosis/status` 的消费者（UI 或蜂鸣/指示灯组件）
- [ ] 决定是否扩展 `DiagnosisMsg`（AIM_READY 等）
- [ ] 如恢复蜂鸣器：新增 `hardware/device/buzzer.hpp` 并在硬件模型接线
- [ ] 评估 `motors` 的 `/alive` 是否改为可选
- [ ] 交叉编译 arm64 + `clang-format --dry-run -Werror`

## 6. 验收与验证

- **编译**：交叉编译通过；`nm -C` 检查 `Diagnosis` 注册。
- **静态**：
  - `grep -rn "diagnosis/status" rmcs_ws/src` 应有生产者与（迁移后）消费者
  - `grep -rn "buzzer" rmcs_ws/src` 在裁决后应为「有」或「明确无」
  - **回归检测**：`grep -rn "buzzer" rmcs_ws/src/rmcs_core/src/controller/` 的缺失应在文档中留档
- **行为验证**：拔掉某个电机（或注入 `/gimbal/pitch/alive = false`），观察 `/diagnosis/status` 变为 `1`（`CTR_MOTOR_OFFLINE`）；注入 `/chassis/supercap/enabled = false` 观察变为 `2`。
- **实车**：上电提示音（若恢复）、掉线提示音/LED。

## 7. 备注与未决问题

- **已核实**：外部 SDK（`https://github.com/JinGangRobo/libjrmcs/releases/download/v3.3.0a7/librmcs-sdk-src-3.3.0-alpha.7-debug.zip`，本地展开于 `rmcs_ws/build-cross-arm64/rmcs_core/_deps/librmcs-src/`）**不包含** `device/buzzer.hpp` 或任何 `BuzzerScore`/`Tone`。`core/include/librmcs/` 仅含 `data/datas.hpp` 与 `spec/{uart,can,gpio}.hpp` + 三个板卡规格。
- **未核实**：`libjrmcs` 的其它版本（或 `dev/juggernaut` / 上游分支）是否提供蜂鸣器 API。
- **未核实**：v3 是否在其它分支（如 `dev/juggernaut`）已有蜂鸣器/诊断 UI 消费方。
- **未核实**：v3 迁移提交中「no buzzer」的具体出处（未在 `dev/jgrobo-ize` 的提交信息里找到该措辞；推断来自迁移记录/PR 描述，需向提交者确认）。已核实的**事实**是：v3 仓库内无任何 buzzer 代码。
- v2 `diagnosis.cpp` 中的 `InputInterface<bool> enable_`、`OutputInterface<double> control_`（L139-140）是**历史死代码**（未注册未使用），迁移时不要带回。
