# 控制器-PID与滤波与数学工具

- 状态：待办
- 优先级：P1（但被 08/09/10 依赖，接口决策会影响 P0 主题）
- 影响机型：全部
- 依赖主题：无（本主题为底座）
- 被依赖主题：08 底盘、09 云台、10 发射、12 诊断
- 验收方式：交叉编译 arm64 通过；`clang-format` 通过；`error_pid_controller`/`matrix_pid_calculator`/`smart_input` 保持与 v2 逐字节一致（可用 sha1 校验）；`PidController::enable` 恢复后 config 中 `enable:` 键有生产者与消费者

## 0. 本次核对的结论摘要（含对任务提示的一处更正）

1. **`pid_controller.cpp` 有一处实质回退**：v2 的 `PidController` 支持可选 `enable` 接口（失能时 `pid_calculator_.reset()` 并告警）；**v3 完全没有**。而 v3 的 `dual-sentry.yaml`（2 处）、`mec-hero.yaml`（4 处）、`mini-infantry.yaml`（4 处）**仍在写 `enable:` 键** → **10 处死配置**。
2. **`pid_calculator.hpp` 的 `integral()` / `clear_integral()` 在 v2 与 v3 中都已存在**（任务提示中「v3 新增 integral/clear_integral 之类接口」的说法**不成立**——两者都有，v3 只是把定义位置从 `update()` 之前移到之后）。v3 真正新增的是**工厂函数** `pid::make_pid_calculator(node, prefix, kp/ki/kd_default)`。
3. `error_pid_controller.cpp`、`matrix_pid_calculator.hpp`、`smart_input.hpp`：**逐字节相同**（sha1 一致）。
4. 滤波/数学工具整体**只改了命名空间**：`rmcs_core::utility` → `rmcs_core::filter`；`angle_warper.hpp` 由 `rmcs_core::utility::angle_wraper::{wrap_to_pi,wrap_to_2pi}` 改为 `rmcs_core::filter::{wrap_to_pi,wrap_to_2pi}`（**函数名不变，去掉了子命名空间 `angle_wraper`**）。
5. v3 另有 `imu_ekf.hpp`、`alpha_beta_angle_filter.hpp` 两个 v2 没有的滤波器；v3 另有 `pid::FrictionWheelPidRecorder`。
6. v2 的通用工具（`tick_timer`、`eigen_structured_bindings`、`fps_counter`）在两仓库中同属 `rmcs_utility` 包，**不涉及迁移**。

---

## 1. v2（reATRM）现状

### 1.1 PID 组件（`src/core/utils/system_ctrl/rmcs_core/src/controller/pid/`）

| 文件 | 行数 | 说明 |
|---|---|---|
| `pid_controller.cpp` | 64 | `PidController`；支持 `measurement`/`setpoint`/`feedforward`（`SmartInput`）+ **可选 `enable`** |
| `error_pid_controller.cpp` | - | `ErrorPidController` |
| `pid_calculator.hpp` | 64 | `PidCalculator`（`kp/ki/kd`、`integral_min/max`、`integral_split_min/max`、`output_min/max`、`reset()`、`clear_integral()`、`integral()`、`update()`） |
| `matrix_pid_calculator.hpp` | - | `MatrixPidCalculator<N>` |
| `smart_input.hpp` | - | `SmartInput`（参数名或立即数） |

`PidController` 的 `enable` 逻辑（`pid_controller.cpp:28-35`、`44-54`）：
```cpp
if (get_parameter("enable", set_enable_interface_name_))
    register_input(set_enable_interface_name_, enable_);
...
if (enable_.active()) {
    if (!enable_.ready()) {
        RCLCPP_WARN_THROTTLE(..., "set enable interface '%s' not found", ...);
        pid_calculator_.reset();
        return;
    } else if (!*enable_) {
        pid_calculator_.reset();
        return;
    }
}
```
出处：v2 提交 `5f8b7ee (须测试)修复云台在失能状态下控制器仍在异常计算`。

`PidCalculator`（v2）关键行：`reset()` L25、`clear_integral()` L28、`integral()` L30、`update(double)` L32。

### 1.2 滤波/数学（`src/core/utils/system_ctrl/rmcs_core/src/utility/`）

| 文件 | 命名空间 / 符号 |
|---|---|
| `angle_warper.hpp` | `rmcs_core::utility::angle_wraper::wrap_to_pi(double)`、`wrap_to_2pi(double)` |
| `kalman_filter.hpp` | `rmcs_core::utility::{KalmanFilter, ExtendedKalmanFilter}`（L14、L87） |
| `low_pass_filter.hpp` | `rmcs_core::utility::LowPassFilter<int N>` |

v2 控制器对这三者的引用（迁移时须逐个改）：
- `gimbal/dual_yaw_solver.cpp:1-2` → `utility/angle_warper.hpp`、`utility/kalman_filter.hpp`
- `gimbal/dual_yaw_controller.cpp:8` → `utility/angle_warper.hpp`
- `gimbal/two_axis_gimbal_solver.hpp:9` → `utility/low_pass_filter.hpp`
- `chassis/omni_wheel_controller.cpp:18` → `utility/low_pass_filter.hpp`
- `chassis/steering_wheel_controller.cpp:13` → `utility/low_pass_filter.hpp`

---

## 2. v3（jRMCS）现状

### 2.1 PID（`rmcs_ws/src/rmcs_core/src/controller/pid/`）

| 文件 | 日期 | 说明 |
|---|---|---|
| `pid_controller.cpp` | 09-23 | **无 `enable` 支持** |
| `error_pid_controller.cpp` | 09-23 | 与 v2 逐字节相同 |
| `pid_calculator.hpp` | 10-15 | 新增 `make_pid_calculator()`（L67） |
| `matrix_pid_calculator.hpp` | 09-23 | 与 v2 逐字节相同 |
| `smart_input.hpp` | 09-23 | 与 v2 逐字节相同 |
| `friction_wheel_pid_recorder.cpp` | 09-23 | **v3 独有**；参数 `wheels`、`min_setpoint_abs` 等，写 CSV |

v3 `pid_controller.cpp` 关键点（对照 v2）：
- 构造：去掉 `if (get_parameter("enable", ...)) register_input(...)`
- `update()`：直接 `auto err = *setpoint_ - *measurement_; *control_ = *feedforward_ + pid_calculator_.update(err);`（无 enable 分支）
- 成员：去掉 `set_enable_interface_name_`、`InputInterface<bool> enable_`

v3 `pid_calculator.hpp`：
- `make_pid_calculator(rclcpp::Node& node, const std::string& prefix, std::optional<double> kp_default, ...)`：按 `prefix + "kp"/"ki"/"kd"` 声明/读取，并读 `prefix + "integral_min"` 等；使用者为 `gimbal/eccentric_dual_yaw.cpp:272-278`

### 2.2 滤波/数学（`rmcs_ws/src/rmcs_core/src/filter/`）

| 文件 | 命名空间 / 符号 |
|---|---|
| `angle_warper.hpp` | `rmcs_core::filter::{wrap_to_pi, wrap_to_2pi}` |
| `kalman_filter.hpp` | `rmcs_core::filter::{KalmanFilter, ExtendedKalmanFilter}` |
| `low_pass_filter.hpp` | `rmcs_core::filter::LowPassFilter<int N>` |
| `imu_ekf.hpp` | `rmcs_core::filter::ImuEkf`（v3 独有） |
| `alpha_beta_angle_filter.hpp` | `rmcs_core::filter::AlphaBetaAngleFilter`（v3 独有） |

---

## 3. 差异清单

### D1 `pid_controller.cpp`：v3 丢失 `enable`（P0，影响 08/09/10）
- **v2 做法**：见 §1.1
- **v3 做法**：`update()` 无 enable 分支（`pid_controller.cpp:38-41`）
- **差异**（`diff -u` 全文 50 行）：删除 `#include <chrono>`/`<cstdint>`；删除 enable 注册与 3 个分支；删除 `set_enable_interface_name_`/`enable_` 成员
- **影响**：
  1. 云台失能（`/gimbal/*/alive == false`）时 PID 仍继续积分 → 复现 v2 `5f8b7ee` 所修的问题（**回退**）
  2. 配置侧 10 处 `enable:` 键成为死配置：`dual-sentry.yaml:119,131`、`mec-hero.yaml:105,117,126,138`、`mini-infantry.yaml:93,105,116,128`
  3. v3 `dual-infantry.yaml` 已经**不写** `enable`（迁移时被删掉了）
- **风险**：**高**（云台/摩擦轮安全相关）

### D2 `pid_calculator.hpp`：`integral()`/`clear_integral()` 两侧都有（P2，更正）
- **事实**：v2 `pid_calculator.hpp:28`（`clear_integral`）、`:30`（`integral`）；v3 `pid_calculator.hpp:52-53`
- **v3 真正新增**：`make_pid_calculator()` 工厂（v3 `pid_calculator.hpp:67-90`）
- **差异**（`diff -u` 全文 69 行）：新增 include `<optional>`/`<string>`/`<rclcpp/node.hpp>`；移动两个函数位置；新增工厂函数
- **影响**：无功能回退；工厂函数只被 `eccentric_dual_yaw.cpp` 使用
- **风险**：无

### D3 `error_pid_controller.cpp` / `matrix_pid_calculator.hpp` / `smart_input.hpp`（P2）
- **逐字节相同**（`sha1sum`）：
  - `error_pid_controller.cpp` = `8cf6dbc5da2e…`
  - `matrix_pid_calculator.hpp` = `0e56167a5666…`
  - `smart_input.hpp` = `5e8467399520…`
- **风险**：无。**注意**：这三个文件的 include 路径若在 v2 里是 `utility/…`，需一并改 `filter/…`（本组文件不涉及）

### D4 滤波器命名空间迁移（P1，影响 08/09）
- **v2**：`rmcs_core::utility::<X>`，头文件路径 `utility/<x>.hpp`
- **v3**：`rmcs_core::filter::<X>`，头文件路径 `filter/<x>.hpp`
- **diff -u 全文**：
  - `angle_warper.hpp`（18 行）：`namespace rmcs_core::utility::angle_wraper` → `namespace rmcs_core::filter`；**函数名不变**（`wrap_to_pi`/`wrap_to_2pi`），调用点由 `utility::angle_wraper::wrap_to_pi(x)` 变为 `filter::wrap_to_pi(x)`
  - `kalman_filter.hpp`（18 行）：仅命名空间 `utility` → `filter`
  - `low_pass_filter.hpp`（19 行）：仅命名空间 `utility` → `filter`
- **影响**：移植 v2 代码时的机械替换；**易漏点**：`angle_wraper` 子命名空间被删除，不能只改外层
- **风险**：低（编译期可发现）

### D5 v3 独有滤波器（P2，仅记录）
- `filter/imu_ekf.hpp`：`ImuEkf`（v3 用 BMI088 时间戳 EKF 取代 librmcs BMI088 驱动，见 v3 提交 `ac99de6`）
- `filter/alpha_beta_angle_filter.hpp`：`AlphaBetaAngleFilter(dt, alpha, beta)`
- `pid/friction_wheel_pid_recorder.cpp`：`FrictionWheelPidRecorder`（参数 `wheels`、`min_setpoint_abs`；未在任何活跃 config 中被引用）

---

## 4. 迁移方案（以 v3 为骨架）

1. **T11-1（P1，建议先做）** 恢复 `PidController` 的 `enable`
   - 在 v3 `pid_controller.cpp` 照抄 v2 的 `enable` 分支（可选参数、`RCLCPP_WARN_THROTTLE` 提示、`pid_calculator_.reset()`），保持成员名 `set_enable_interface_name_` / `enable_`
   - 这样即可让 v3 已有的 10 处 `enable:` 键重新生效，无需改 config
   - 给 v3 `dual-infantry.yaml` 的 `pitch_angle_err_pid_controller` / `pitch_velocity_pid_controller` 也补上 `enable: /gimbal/pitch/alive`（与 v2 对齐）
2. **T11-2（P1）** `make_pid_calculator` 保持现状（v3 独有），不做删除；文档中标注仅 `eccentric_dual_yaw` 使用
3. **T11-3（P1）** 建立「命名空间替换」清单（供 08/09/10 复用）：
   - `#include "utility/angle_warper.hpp"` → `#include "filter/angle_warper.hpp"`
   - `#include "utility/kalman_filter.hpp"` → `#include "filter/kalman_filter.hpp"`
   - `#include "utility/low_pass_filter.hpp"` → `#include "filter/low_pass_filter.hpp"`
   - `utility::angle_warper::wrap_to_pi` → `filter::wrap_to_pi`；`utility::angle_warper::wrap_to_2pi` → `filter::wrap_to_2pi`
   - `utility::LowPassFilter<...>` → `filter::LowPassFilter<...>`
   - `utility::ExtendedKalmanFilter<...>` → `filter::ExtendedKalmanFilter<...>`
4. **T11-4（P2）** 校验 `error_pid_controller` / `matrix_pid_calculator` / `smart_input` 保持与 v2 一致（sha1 对比，纳入 CI 或本文档人工核对）
5. **T11-5（P2）** 评估 `FrictionWheelPidRecorder` 是否接入摩擦轮整定流程（依赖 10）

---

## 5. 执行步骤 checklist

- [ ] `pid_controller.cpp` 恢复 `enable` 可选接口（照抄 v2 逻辑）
- [ ] `dual-infantry.yaml` 补 `enable: /gimbal/pitch/alive`（2 处）
- [ ] 确认 `dual-sentry/mec-hero/mini-infantry` 的 `enable:` 键随 T11-1 生效
- [ ] 输出「`utility::` → `filter::`」替换清单给 08/09/10 使用
- [ ] sha1 复核 3 个逐字节相同的 PID 文件
- [ ] 交叉编译 arm64 + `clang-format --dry-run -Werror`

## 6. 验收与验证

- **编译**：交叉编译通过。
- **sha1 校验**：
  ```bash
  for f in controller/pid/error_pid_controller.cpp controller/pid/matrix_pid_calculator.hpp controller/pid/smart_input.hpp; do
    sha1sum "/workspaces/reATRM/src/core/utils/system_ctrl/rmcs_core/src/$f" \
            "/workspaces/RMCS/rmcs_ws/src/rmcs_core/src/$f"
  done
  ```
  三者两行应分别一致。
- **配置一致性**：脚本检查所有 `enable:` 键在 v3 中都有 `get_parameter("enable")` 消费者（T11-1 完成后应为 1 处消费者、10 处生产者键）。
- **行为验证**：把 `/gimbal/pitch/alive` 置 false，观察 `/gimbal/pitch/control_torque` 归零且未积分（对比恢复前后）。
- **单元测试（推荐）**：`PidCalculator` 的 `integral_split_min/max` 分段积分与 `output_min/max` 钳位做数值用例。

## 7. 备注与未决问题

- **更正记录**：任务提示「注意 v3 新增 integral/clear_integral 之类接口」——经逐行核对，**v2 与 v3 都有**这两个接口；v3 新增的是 `make_pid_calculator()` 工厂。本文档以此为准。
- **未核实**：v2 是否在其它分支/机型中把 `enable` 换成别的机制（例如 `ErrorPidController` 里是否也有失能逻辑）。`error_pid_controller.cpp` 两侧逐字节相同 → 无该机制。
- **未核实**：v3 上游是否因 `rmcs_utility/rclcpp/node_mixin` 的引入而计划用别的方式替代 `enable`（当前无任何替代消费者）。
- v3 `pid_calculator.hpp` 依赖 `<rclcpp/node.hpp>`，使该头文件从「纯数学」变为「依赖 ROS」，`matrix_pid_calculator.hpp` / `smart_input.hpp` 仍为纯头；如需把 `pid_calculator` 复用为非 ROS 场景，需拆分工厂函数。
