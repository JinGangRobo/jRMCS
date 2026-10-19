# 16-通用工具-rmcs_utility

- 状态：待办
- 优先级：P1（v3 工具集是 v2 的超集，风险集中在 `RingBuffer` API 改名/收紧与"v2 工具散落在 librmcs"的搬迁）
- 影响机型：全部；影响集中在移植 v2 硬件模型（主题 06/07）与设备驱动（主题 05）
- 依赖主题：01-仓库布局与包结构、02-工程链路（C++ 标准）、03-组件运行时（executor 依赖 `ring_buffer`/`tdigest`/`thread_config`）、05-硬件栈、17-视觉子系统
- 验收方式：① 交叉编译 arm64 全量通过；② `grep -rn "rmcs_utility/" rmcs_ws/src` 的每个引用点均可解析；③ 移植后的硬件模型实机收包（裁判系统串口）无丢帧/无编译告警；④ `clang-format --dry-run -Werror`

## 1. v2（reATRM）现状

包路径：`/workspaces/reATRM/src/core/utils/system_ctrl/rmcs_utility`

文件清单（7 个头文件，全部在 `include/rmcs_utility/`）：

| 文件 | 行数 | 使用者（v2） |
|---|---|---|
| `crc/dji_crc.hpp` | 120 | `rmcs_core` 设备驱动（CAN 校验） |
| `double_buffer.hpp` | 51 | `rmcs_core/src/hardware/device/hipnuc.hpp:9` |
| `eigen_structured_bindings.hpp` | 44 | `rmcs_core` 数学/滤波代码（3 处 include） |
| `fps_counter.hpp` | 38 | `hardware/model/{dual_infantry,dual_sentry,hero,mini_infantry,pid_tune_tool}.cpp`、`controller/shooting/friction_wheel_controller.cpp:177` |
| `package_receive.hpp` | 90 | `rmcs_core/src/hardware/device/hipnuc.hpp:10`、`rmcs_core/src/referee/status.cpp:10` |
| `ring_buffer.hpp` | 190 | **v2 本包内几乎无人使用**；v2 硬件模型用的是 librmcs 自带的 `librmcs::utility::RingBuffer`（见下） |
| `tick_timer.hpp` | 22 | 设备驱动（`lk_motor.hpp`、`dm_motor.hpp`、`dji_motor.hpp`、`supercap.hpp`）、UI（4 个机型）、`referee/status.cpp`、底盘控制器 |

- `CMakeLists.txt:6`：`set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")`；`package.xml` 无依赖只需 ament。
- **v2 的另一处工具集**：`/workspaces/reATRM/src/core/utils/system_ctrl/rmcs_core/librmcs/librmcs/utility/` 内嵌了 `cross_os.hpp`、`ekf_imu.hpp`、`endian_promise.hpp`、`logging.hpp`、`pid_calculator.hpp`、`ring_buffer.hpp`（命名空间 `librmcs::utility`）。v2 硬件模型用的是这一份：
  - `rmcs_core/src/hardware/model/dual_infantry.cpp:326,407,428`、`dual_sentry.cpp:329,412,435`、`hero.cpp:305,391,417`、`mini_infantry.cpp:298,385,411` 使用 `librmcs::utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256}` 与 `pop_front_multi` / `emplace_back_multi`。
  - 这两份 `RingBuffer`（`rmcs_utility::RingBuffer` 与 `librmcs::utility::RingBuffer`）实现不同（未核实是否逐字节相同，文件名相同但命名空间不同）。
- v2 本包内 `ring_buffer.hpp` 的公开 API（`ring_buffer.hpp:14-167`）：`RingBuffer(size_t)`、`max_size()`、`readable()`、`writeable()`、`front()`、`back()`、`emplace_back_multi(F, count=-1)`、`emplace_back(Args&&...)`、`push_back_multi(F, count=-1)`、`push_back(const T&)`、`push_back(T&&)`、`pop_front_multi(F, count=-1)`、`pop_front(F&&)`、`clear()`；`std::atomic_signal_fence(release)` + `in_.store(out_.store) release`。

## 2. v3（jRMCS）现状

包路径：`/workspaces/RMCS/rmcs_ws/src/rmcs_utility`

文件清单（15 个头文件 + 构建文件）：

| 文件 | 行数 | 与 v2 关系 | 使用者（v3） |
|---|---|---|---|
| `crc/dji_crc.hpp` | 120 | **逐字节相同** | 设备驱动 |
| `double_buffer.hpp` | 51 | **逐字节相同** | `rmcs_core/src/hardware/device/hipnuc.hpp` |
| `eigen_structured_bindings.hpp` | 44 | **逐字节相同** | `rmcs_core` 数学代码（5 处） |
| `fps_counter.hpp` | 38 | **逐字节相同** | `controller/shooting/friction_wheel_controller.cpp:171`、`hero_friction_wheel_controller.cpp:191` |
| `package_receive.hpp` | 90 | **逐字节相同** | `hipnuc.hpp`、`referee/status.cpp:10` |
| `tick_timer.hpp` | 22 | **逐字节相同** | 设备驱动（`dm_motor`/`dji_motor`/`j_supercap`）、`referee/status.cpp:11`、`controller/chassis/j_omni_wheel_controller.cpp:14` |
| `ring_buffer.hpp` | 532 | **重写/大幅扩展**（190 → 532 行） | `rmcs_executor/component.hpp:250`、`rmcs_core` 硬件模型（`dual_sentry.cpp:309,379`、`hero.cpp:270,343`、`mini_infantry.cpp:279,366`）、`hardware/device/vt13.hpp:308`、`rmcs_auto_aim_v2`（`capturer.cpp`、`imu_snapshot_buffer.hpp`） |
| `atomic_futex.hpp` | 160 | **v3 新增** | `rmcs_auto_aim_v2/src/io/capturer/capturer.cpp` |
| `csv_writer.hpp` | 105 | **v3 新增** | `rmcs_core/src/identification/swept_frequency_controller.cpp`、`static_torque_test_controller.cpp` |
| `endian_promise.hpp` | 201 | **v3 新增（从 v2 `librmcs` 迁入并重写）** | `rmcs_core/src/hardware/device/dji_motor.hpp:19` |
| `memory_pool.hpp` | 196 | **v3 新增** | 仅 `pooled_shared_factory.hpp` |
| `pooled_shared_factory.hpp` | 76 | **v3 新增** | `rmcs_auto_aim_v2/src/io/capturer/capturer.cpp` |
| `rclcpp/node_mixin.hpp` | 54 | **v3 新增** | `rmcs_core/src/debug/value_collector.cpp`、`controller/chassis/chassis_controller.cpp`、`controller/chassis/sentry_climber.cpp` |
| `tdigest.hpp` | 660 | **v3 新增** | `rmcs_executor/src/executor.hpp`（性能分位数统计） |
| `thread_config.hpp` | 318 | **v3 新增** | `rmcs_executor/src/executor.hpp`（`thread_config` 参数） |

- `CMakeLists.txt:6-7`：`set(CMAKE_CXX_STANDARD 23)` + `CMAKE_CXX_STANDARD_REQUIRED ON`（v2 为 `-std=c++20`）；`package.xml` **无差异**。
- v3 引用的 12 个工具头统计（`grep -c`）：`ring_buffer` 7、`tick_timer` 5、`eigen_structured_bindings` 5、`rclcpp/node_mixin` 3、`crc/dji_crc` 3、`package_receive` 2、`fps_counter` 2、`csv_writer` 2、`pooled_shared_factory`/`endian_promise`/`atomic_futex`/`thread_config`/`tdigest`/`memory_pool` 各 1。
- v3 **没有** `librmcs` 源码目录：`rmcs_core/CMakeLists.txt:18-25,39` 通过 `FetchContent` 拉取 `libjrmcs` 预编译 SDK（`librmcs-sdk`），因此 v2 的 `librmcs::utility::*` 工具在 v3 侧全部改走 `rmcs_utility`（详见主题 05）。

### 2.1 v3 新增头文件要点

- `endian_promise.hpp`：`swap_endian<T>()`（`bit_cast` + `std::array<std::byte>`，2/4/8 字节），命名空间 **`rmcs_utility`**（v2 等价物在 `librmcs::utility`，见 3.3）。
- `memory_pool.hpp`：`MemoryPool<Size, Align>` 定容槽池（非线程安全；析构时仍有未归还槽会被 `std::fprintf` 报错并 `未核实` 是否 terminate）。
- `pooled_shared_factory.hpp`：`PooledSharedFactory<T>`（`make()` / `try_make()` / `max_size()`），在池上构造 `shared_ptr`，容量耗尽返回 `nullptr` 或抛 `bad_alloc`；供视觉高频帧对象复用。
- `atomic_futex.hpp`：基于 `linux/futex.h` + `syscall(SYS_futex)` 的原子等待/唤醒（`atomic_futex_address`、`atomic_futex_wait_until_steady` 等），`detail` 命名空间。
- `thread_config.hpp`：`ThreadConfig{spec[, default_name]}`，解析 `key=value;...`（`name`/`cpus`/`policy`/`priority`/`nice`），`apply_to_current_thread()` 返回 `std::expected<void, std::string>`，用 `pthread_setschedparam` / `setpriority` / `pthread_setaffinity_np` / `pthread_setname_np`；依赖 `<expected>` 与 `<format>`（C++23）。
- `tdigest.hpp`：`TDigest<T>` 分位数摘要，**派生自 Apache-2.0 的 digestible 项目**（文件头有完整出处与许可声明，含 "Modifications for RMCS by Zihan Qin, 2026"）。移植/复制该文件时必须保留许可头。
- `csv_writer.hpp`：`CsvWriter`（`open()` 自动建父目录、`close()` 析构关闭、不可拷贝/移动）。
- `rclcpp/node_mixin.hpp`：`NodeMixin`（`node` 别名、`static options()` = `automatically_declare_parameters_from_overrides(true)`、`info/warn/error` 走 `std::format`、`param()`（缺失即打错误日志并抛 `lack of param`）/ `param_or()`）。**注意其基类式 CRTP 风格**：v3 组件按 `class X : Component, rclcpp::Node, rmcs_utility::NodeMixin` 混入。

## 3. 差异清单

### 3.1 逐文件结论（`diff -u` / `md5sum` 实测）

| # | 文件 | v2 | v3 | 结论 |
|---|---|---|---|---|
| 1 | `crc/dji_crc.hpp` | 有 | 有 | **逐字节相同**（md5 一致） |
| 2 | `double_buffer.hpp` | 有 | 有 | **逐字节相同** |
| 3 | `eigen_structured_bindings.hpp` | 有 | 有 | **逐字节相同** |
| 4 | `fps_counter.hpp` | 有 | 有 | **逐字节相同** |
| 5 | `package_receive.hpp` | 有 | 有 | **逐字节相同** |
| 6 | `tick_timer.hpp` | 有 | 有 | **逐字节相同** |
| 7 | `ring_buffer.hpp` | 190 行 | 532 行 | **重写 + API 改名 + 约束收紧**（见 3.2） |
| 8 | `atomic_futex.hpp` | 无 | 160 行 | v3 新增 |
| 9 | `csv_writer.hpp` | 无 | 105 行 | v3 新增 |
| 10 | `endian_promise.hpp` | 无（在 `librmcs/utility/`） | 201 行 | v3 新增（迁移 + 重写，见 3.3） |
| 11 | `memory_pool.hpp` | 无 | 196 行 | v3 新增 |
| 12 | `pooled_shared_factory.hpp` | 无 | 76 行 | v3 新增 |
| 13 | `rclcpp/node_mixin.hpp` | 无 | 54 行 | v3 新增 |
| 14 | `tdigest.hpp` | 无 | 660 行 | v3 新增（Apache-2.0 派生） |
| 15 | `thread_config.hpp` | 无 | 318 行 | v3 新增 |
| — | `CMakeLists.txt` | `-std=c++20` | `CMAKE_CXX_STANDARD 23` + `REQUIRED ON` | 构建差异 |
| — | `package.xml` | — | — | **逐字节相同** |
| — | `LICENSE` | 有 | 有 | `未核实` 是否逐字节相同（未做 diff） |

**v2 独有、v3 缺失的头文件：无**（v2 本包 7 个文件在 v3 全部存在）。

### 3.2 `ring_buffer.hpp` 详细差异（唯一有实质冲突的文件）

API 对照：

| 语义 | v2（`ring_buffer.hpp`） | v3（`ring_buffer.hpp`） | 兼容性 |
|---|---|---|---|
| 容量剩余 | `writeable()`（:42） | `writable()`（:248） | ❌ 改名 |
| 取首元素 | `front()`（:54） | `peek_front()`（:285） | ❌ 改名（返回类型同为 `T*`） |
| 取尾元素 | `back()`（:70） | `peek_back()`（:337） | ❌ 改名 + **语义修正**：v2 返回 `storage_[in & mask]`（越界槽，实为"下一个写位"，可疑），v3 返回 `storage_[(in - 1) & mask]`（真正的最后元素） |
| 批量构造 | `emplace_back_multi(F, count=-1)`（:81） | `emplace_back_n(F, count=numeric_limits<size_t>::max())`（:358） | ❌ 改名（默认值语义等价） |
| 批量生成入队 | `push_back_multi(F generator, count=-1)`（:113） | `push_back_n(F generator, count=...)`（:405） | ❌ 改名 |
| 批量出队 | `pop_front_multi(F, count=-1)`（:127） | `pop_front_n(F, count=...)`（:443） | ❌ 改名 |
| 单元素 | `push_back(copy/move)`、`emplace_back`、`pop_front`、`clear()`、`readable()`、`max_size()` | 同名（:417/425/387/477/503/236/228） | ✅ |
| 新增 | — | `BasicIterator<is_const>`（随机访问迭代器，:24-141）、`ReadableView<is_const>` / `readable_view()` / `const_readable_view()`（:146-192、:259-277）、`peek_front_n()`（:304）、`pop_front_until(const_iterator)`（:486） | v3 新增能力 |
| 拷贝/移动 | 未显式删除（隐式依赖 `atomic` 成员而不可拷贝） | **显式 `= delete`**（:210-213） | 更明确 |
| 存储 | `std::byte data[sizeof(T)]` 嵌套 `struct Storage`（:185-187） | 同名（:526-528） | ✅ |
| 内存序 | `std::atomic_signal_fence(release)` + release store（:100-101、150-151） | 直接 release store（:377、465） | 行为等价（更强/更简） |
| 函数约束 | 仅 `requires requires(F f, T t) { f(std::move(t)); }`（无 `noexcept`） | `requires requires(F& f, T& t) { { f(std::move(t)) } noexcept; }`，以及 `{ f(storage) } noexcept` / `{ { f() } noexcept; { T{f()} } noexcept; }` | ⚠️ **收紧**：回调/构造仿函数必须是 `noexcept`，否则**编译失败** |

影响与风险：

1. **改名 → 编译失败**：v2 硬件模型大量使用 `pop_front_multi` / `emplace_back_multi`（如 `dual_infantry.cpp:326,407`），移植到 v3 必须改为 `pop_front_n` / `emplace_back_n`（v3 现有写法见 `rmcs_core/src/hardware/dual_sentry.cpp:309,379`）。
2. **`noexcept` 收紧 → 编译失败**：v3 现有调用点已显式加 `noexcept`（`dual_sentry.cpp:310` `[&buffer](std::byte byte) noexcept`；`:380` `[&uart_data](std::byte* storage) noexcept`）。移植 v2 代码时**必须**给回调加 `noexcept`。
3. **`back()` 语义修正**：v2 的 `back()` 返回值可疑（`in & mask` 是"下一个写位"）。已核实 v2/v3 **均无调用点**（`grep` 未命中 `.back()` 使用），故无实际影响。
4. **`RingBuffer` 来源不同（重要）**：v2 硬件模型用的是 `librmcs::utility::RingBuffer`（`librmcs` 内嵌头），v3 统一为 `rmcs_utility::RingBuffer`。因此**不能**直接照搬 v2 的 include 与命名空间；且 v2 本包 `rmcs_utility::RingBuffer` 在 v2 中几乎无用例，说明该文件在 v2 属于"未被使用的旧实现"，v3 才是被广泛使用的版本。
5. `RingBuffer` 现在是 **SPSC**（单生产者单消费者）语义；v3 在 `component.hpp:250` 用它做事件队列，并要求 `T` `nothrow_copy_constructible` + `nothrow_destructible`（`component.hpp:158-160`）。

### 3.3 `endian_promise.hpp`：跨包搬迁 + 重写

| 项 | v2 | v3 |
|---|---|---|
| 路径 | `rmcs_core/librmcs/librmcs/utility/endian_promise.hpp` | `rmcs_utility/include/rmcs_utility/endian_promise.hpp` |
| 命名空间 | `librmcs::utility` | `rmcs_utility` |
| 依赖 | 额外 `#include "../utility/cross_os.hpp"` | 无（去掉 cross_os） |
| 实现 | 逐字节手工索引 `((uint8_t*)&result)[i]` | `std::bit_cast<std::array<std::byte, N>>` + `static_assert(2/4/8 字节)` |
| 使用点 | 设备驱动经 librmcs | `rmcs_core/src/hardware/device/dji_motor.hpp:19` `#include <rmcs_utility/endian_promise.hpp>` |

`diff -u` 共 271 行（含重写）。迁移结论：v2 的 `librmcs` 内置工具在 v3 由 `rmcs_utility` 等价替代，**移植 v2 设备驱动时把 `librmcs::utility::xxx` 换为 `rmcs_utility::xxx` 并核对 API**。

### 3.4 结构性差异

| 差异点 | v2 | v3 | 影响/风险 |
|---|---|---|---|
| 包内文件数 | 7 | 15 | 移植期"找不到头文件"的排查方向：v3 侧工具更全，缺的只有 librmcs 内置工具 |
| C++ 标准 | `-std=c++20` | `CMAKE_CXX_STANDARD 23` | v3 新头用 `<expected>`、`<format>`、`std::bit_cast`、`if consteval` 风格；`AGENTS.md` 写 C++20 与实现不符（同主题 03） |
| 许可 | `LICENSE` 存在 | `LICENSE` 存在 + `tdigest.hpp` 携带 Apache-2.0 声明 | 引入 `tdigest.hpp` 需保留声明 |
| v2 内嵌 librmcs 工具（`cross_os`/`ekf_imu`/`logging`/`pid_calculator`/`ring_buffer`/`endian_promise`） | 在 `rmcs_core/librmcs/...` | 由外部 `librmcs-sdk`（FetchContent）+ `rmcs_utility` 分担 | 具体归属归主题 05；本主题仅记录 `endian_promise`/`ring_buffer` 的搬迁 |

## 4. 迁移方案（以 v3 为骨架）

**总原则：v3 `rmcs_utility` 为唯一工具真源，不把 v2 的 `rmcs_utility::RingBuffer` 版本带回 v3；移植 v2 代码时按改名/`noexcept` 适配。**

1. **不动 v3 的 15 个头文件**（`ring_buffer.hpp` 保持 532 行版本）。`ring_buffer` 已被 executor、`rmcs_core` 硬件模型、视觉子系统共同依赖（7 处），改回 v2 API 会引发连锁修改，不划算。
2. **移植 v2 组件/硬件代码时的机械替换表**（写成迁移时的核对清单）：
   | v2 写法 | v3 写法 |
   |---|---|
   | `rmcs_utility::RingBuffer<T> b{n}`（若存在） | 同 |
   | `librmcs::utility::RingBuffer<T> b{n}` | `rmcs_utility::RingBuffer<T> b{n}`（+ `#include <rmcs_utility/ring_buffer.hpp>`） |
   | `.writeable()` | `.writable()` |
   | `.front()` / `.back()` | `.peek_front()` / `.peek_back()` |
   | `.emplace_back_multi(f, n)` | `.emplace_back_n(f, n)` |
   | `.push_back_multi(f, n)` | `.push_back_n(f, n)` |
   | `.pop_front_multi(f, n)` | `.pop_front_n(f, n)` |
   | 回调/构造仿函数（无 `noexcept`） | 全部加 `noexcept` |
   | `#include <librmcs/utility/endian_promise.hpp>` / `librmcs::utility::swap_endian` | `#include <rmcs_utility/endian_promise.hpp>` / `rmcs_utility::swap_endian` |
3. **参数读取统一到 `NodeMixin`**：移植 v2 组件时把 `declare_parameter` + `get_parameter` 手写模式改为 `rmcs_utility::NodeMixin` 的 `node::param()` / `node::param_or()`（缺失即报错，避免 v2 式静默用默认值）；参考 v3 现有 `rmcs_core/src/debug/value_collector.cpp:25-39`。
4. **新增工具的使用建议**：
   - 移植 v2 的辨识/激励工具（主题 11）时用 `csv_writer.hpp` 替代 v2 自写 CSV（v3 已有 `identification/*` 用例）。
   - 移植需要实时线程的组件时用 `thread_config.hpp`（经 executor 的 `thread_config` 参数，见主题 03）。
   - 视觉/高频帧路径沿用 `memory_pool` + `pooled_shared_factory` + `atomic_futex`（主题 17），不要引入 v2 的裸 `new` 模式。
5. **不做的事**：不删除 v3 任何工具头；不回退 C++23；不把 v2 的 `ring_buffer` 实现（190 行）作为 v3 的 `RingBuffer` 备选。
6. **许可**：若最终把 `tdigest.hpp` 纳入分发产物，确认在仓库 `LICENSE`/文档中保留 Apache-2.0 归属（v3 现状已保留文件头声明）。

## 5. 执行步骤 checklist

- [ ] 冻结 v3 `rmcs_utility` 15 个文件为基线，不做"回退式"修改（16-1）
- [ ] 在主题 06/07 移植 v2 硬件模型时，逐处替换 `pop_front_multi`/`emplace_back_multi`/`writeable`/`front`/`back`（16-2）
- [ ] 同批移植处给所有 RingBuffer 回调补 `noexcept`（16-3）
- [ ] 把 v2 的 `librmcs::utility::*`（含 `endian_promise`、`ring_buffer`）引用改写为 `rmcs_utility::*`（16-4，联动主题 05）
- [ ] 复核移植代码中 `RingBuffer` 的 `T` 是否满足 `is_nothrow_copy_constructible` / `is_nothrow_destructible`（仅事件队列需要）（16-5）
- [ ] 移植组件参数读取改用 `NodeMixin`（16-6，与 03-5 合并执行）
- [ ] 交叉编译 arm64 全量通过（16-7）
- [ ] 记录 `tdigest.hpp` 的 Apache-2.0 许可归属（16-8）

## 6. 验收与验证

- 编译：`./.script/build-rmcs-cross --target-arch=arm64 --packages-skip rmcs_auto_aim_v2 --executor sequential`。
- 静态自查：
  - `grep -rn "writeable\|_multi(\|\.front()\|\.back()" rmcs_ws/src --include=*.hpp --include=*.cpp` 应无命中（除 `peek_front`/`peek_back`/`front()` 的合法用途需人工确认）。
  - `grep -rn "librmcs::utility" rmcs_ws/src` 应无命中。
  - `grep -rn "rmcs_utility/" rmcs_ws/src` 的每个路径都存在于 `rmcs_utility/include/rmcs_utility/`。
- 运行期（实机，dual-infantry 优先）：裁判系统串口收包（`referee_ring_buffer_receive_`）不应出现丢帧或队列满日志；executor 5 s 统计中 `Skipped` 为 0 或极低。
- 格式：`clang-format --dry-run -Werror`（仓库根）。

## 7. 备注与未决问题

- `rmcs_utility/LICENSE` 与 v2 是否逐字节相同：**未核实**（未做 diff）。
- v2 的 `librmcs::utility::RingBuffer` 与 v2 `rmcs_utility::RingBuffer` 是否实现相同：**未核实**（两者都叫 `RingBuffer`，若相同则说明 v2 存在重复实现，v3 收敛为一份）。
- `memory_pool.hpp` 析构时"仍有未归还槽"的确切行为（报错后是否 `std::terminate`）：本次只读到头文件的 `std::fprintf` 分支，**未完整核实**。
- `atomic_futex.hpp` 依赖 `linux/futex.h`，仅 Linux；交叉编译目标为 arm64 Linux，已通过（现状构建产物存在），无需额外处理。
- v3 未使用的 v3 新头（如 `memory_pool` 仅被 `pooled_shared_factory` 使用）是否属于"预留能力"：属 v3 现状，本主题不做删减建议。
- `AGENTS.md` 中"C++20"与实际 `C++23` 的不一致，建议在主题 02 统一修正，本主题仅记录。
