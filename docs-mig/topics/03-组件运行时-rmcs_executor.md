# 03-组件运行时-rmcs_executor

- 状态：待办
- 优先级：P0
- 影响机型：全部（dual-infantry 为 P0 重点，dual-sentry / mini-infantry / mec-hero 同等适用）
- 依赖主题：01-仓库布局与包结构、02-工程链路-构建与容器与CI（C++ 标准）、16-通用工具-rmcs_utility（`ring_buffer` / `tdigest` / `thread_config`）
- 验收方式：① 交叉编译 `build-rmcs-cross --target-arch=arm64 --packages-skip rmcs_auto_aim_v2` 通过；② 插件注册核验 `nm -C install-cross-arm64/lib/librmcs_core.so | grep registerPlugin<...>`；③ `clang-format --dry-run -Werror`；④ 离线静态检查（`-fsyntax-only` 复用 `compile_commands.json`）；⑤ 实机启动后观察 executor 每 5 s 统计日志与接口配对日志

> 本主题只比对 `rmcs_executor` 包本身（`component.hpp` / `component.cpp` / `executor.hpp` / `main.cpp` / `predefined_msg_provider.hpp` + 构建文件）。
> executor 的**下游使用者**（v2 的 `rmcs_core` / `autopilot_client` / `rmcs_auto_aim_v2` 组件）在各自主题（05–13、17）内详述，本文件只列与 API 兼容性直接相关的影响。

## 1. v2（reATRM）现状

包路径：`/workspaces/reATRM/src/tasks/system_ctrl/rmcs_executor`

文件清单（与 v3 同名同结构）：

| 文件 | 行数 | 作用 |
|---|---|---|
| `include/rmcs_executor/component.hpp` | 188 | `Component` 基类、`InputInterface<T>` / `OutputInterface<T>`、注册函数 |
| `src/component.cpp` | 7 | 只有 `Component::initializing_component_name` 定义 |
| `src/executor.hpp` | 186 | `Executor`：接口配对 + 依赖排序 + 1 kHz 循环线程 |
| `src/main.cpp` | 72 | pluginlib 加载组件 + SIGSEGV handler |
| `src/predefined_msg_provider.hpp` | 22 | `/predefined/update_rate`、`/predefined/update_count`、`/predefined/timestamp` |
| `CMakeLists.txt` / `package.xml` / `README.md` | — | 构建 |

关键事实（含行号）：

- `Component` 基类（`component.hpp:15-31`）：`friend class Executor`；拷贝/移动全部 `= delete`；虚函数 `before_pairing(const std::map<std::string, const std::type_info&>&)`（:26）、`before_updating()`（:29）、纯虚 `update()`（:31）。
- `InputInterface<T>`（:33-90）：`make_and_bind_directly()`、`bind_directly()`、`operator->` / `operator*`；私有 `void** activate()`（:81-84）。
- `OutputInterface<T>`（:92-126）：`activate(Args&&...)` 用 placement-new 构造到 `std::aligned_storage_t<sizeof(T), alignof(T)> data_`（:124），析构用 `std::destroy_at(std::launder(...))`。
- 注册接口（:130-144）：
  - `register_input<T>(name, InputInterface<T>&, bool required = true)`
  - `register_output<T, Args...>(name, OutputInterface<T>&, Args&&...)`
- `create_partner_component<T>(name, args...)`（:146-154）会把 `initializing_component_name` 设为 `name.c_str()`。
- `static const char* initializing_component_name;`（:156）；`Component()` 用 `initializing_component_name` 初始化 `component_name_`（:159-160）。
- 内部声明结构（:165-178）：`InputDeclaration{type, name, required, void** pointer_to_data_pointer}`、`OutputDeclaration{type, name, void* data_pointer, Component* component}`。
- `Executor`（`executor.hpp:20-185`）：
  - 构造（:22-31）注册 `PredefinedMsgProvider`，`component_name` 固定 `"predefined_msg_provider"`。
  - 析构（:32-35）只 `thread_.join()`。
  - `add_component(std::shared_ptr<Component>)` **按值**接收（:37-43），递归加入 partner 组件；`rclcpp::Node` 组件加入 `SingleThreadedExecutor`。
  - `start()`（:45-69）：`init()` → 所有组件 `before_updating()` → 取参数 `update_rate`（取不到即抛 `Unable to get parameter update_rate<double>`）→ 起线程，循环体固定为：
    ```cpp
    predefined_msg_provider_->set_timestamp(next_iteration_time);
    next_iteration_time += period;                       // :62
    for (const auto& component : updating_order_)
        component->update();
    std::this_thread::sleep_until(next_iteration_time);
    ```
    即**无浮点校验、无跳帧补偿、无异常捕获、无统计、无线程属性配置**；时间戳用**计划时间**（下一周期目标点）而非实际开始时间。
  - `init()`（:72-155）：`unordered_map<string, OutputDeclaration*>` 检测重名输出（`Duplicate names of output: <name>`，:82）；`before_pairing(user_output_map)` 传入 `std::map<std::string, const std::type_info&>`（:76、:88）；按名字配对输入/输出并校验 `typeid`（:107-117）；`dependency_count_` + `wanted_by_` 拓扑排序（:119-135、:157-174），环形依赖时 FATAL + 抛异常（:136-154）。
- `main.cpp`：
  - `segmentation_fault_handler(int)`（:15-31）：`backtrace()` + `backtrace_symbols_fd(array + 2, ...)`，打印 `[Fatal] Segmentation fault` / `>>> STACK TRACE BEGIN` / `<<< STACK TRACE END` 后 `exit(1)`；在 `main` 首行 `std::signal(SIGSEGV, segmentation_fault_handler);`（:34）。
  - 插件加载（:38-39）：`pluginlib::ClassLoader<rmcs_executor::Component> component_loader("rmcs_executor", "rmcs_executor::Component");`
  - 参数 `components` 缺失时**显式报错**：`throw std::runtime_error("Missing parameter 'components' or config is not found")`（:46-47）。
  - 组件描述正则 `R"(\s*(\S+)\s*->\s*(\S+)\s*)"`（:49）；`initializing_component_name = component_name.c_str()`（:64）。
- 构建：`CMakeLists.txt:6` `set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")`；`package.xml` 只有 `rclcpp` + `pluginlib`（无 `rmcs_utility`）。
- plugins.xml：**本包没有** `plugins.xml`；插件清单在 `rmcs_core` 等提供组件的包里（`/workspaces/reATRM/src/core/utils/system_ctrl/rmcs_core/plugins.xml`、`.../rmcs_auto_aim_v2/plugins.xml`、`.../autopilot_client/plugins.xml`）。
- `predefined_msg_provider.hpp`：`register_output("/predefined/update_rate", update_rate_)`、`register_output("/predefined/update_count", update_count_, static_cast<size_t>(-1))`、`register_output("/predefined/timestamp", timestamp_)`；`update()` 只 `*update_count_ += 1;`。
- v2 组件侧对 executor API 的用法（影响面）：`before_pairing` 只有 1 处覆写（`rmcs_core/src/broadcaster/value_broadcaster.cpp:22`）；`create_partner_component` 6 处；`register_input` / `register_output` 在 `rmcs_core` 中 355 处（`grep -c`）。

## 2. v3（jRMCS）现状

包路径：`/workspaces/RMCS/rmcs_ws/src/rmcs_executor`

| 文件 | 行数 | 与 v2 关系 |
|---|---|---|
| `include/rmcs_executor/component.hpp` | 556 | **大幅扩展**（事件接口、类型擦除绑定、注册重名校验） |
| `src/component.cpp` | 7 | 1 行类型变化 |
| `src/executor.hpp` | 646 | **大幅扩展**（统计、跳帧、异常、线程配置、事件生命周期） |
| `src/main.cpp` | 72 | 2 处改动 |
| `src/predefined_msg_provider.hpp` | 22 | **逐字节相同**（`diff -u` 为空） |
| `CMakeLists.txt` | 33 | C++23、`CMAKE_CXX_STANDARD_REQUIRED ON` |
| `package.xml` | — | 新增 `<depend>rmcs_utility</depend>` |
| `README.md` | — | **逐字节相同** |

关键新增（含行号）：

- 接口种类枚举与事件状态（`component.hpp:26-42`）：`enum class InterfaceKind { Normal, Event }`、`interface_kind_name()`、`using EventState = std::uint32_t`、`EVENT_DISABLED_BIT`、`EVENT_ACTIVE_MASK`。
- `Component::OutputInfo{std::reference_wrapper<const std::type_info> type; InterfaceKind kind;}`、`using OutputInfoMap = std::map<std::string, OutputInfo>`（:48-52）。
- `before_pairing(const OutputInfoMap&)`（:61）——签名变化。
- 绑定改为**类型擦除函数指针**：`InputDeclaration{type, name, kind, required, void* binding, BindFunction bind}`（:526-533）、`OutputDeclaration{..., OutputLifecycleHook enable/disable/wait_idle, Component* component}`（:535-545）；`bind_input_interface<T>`（:498-501）、`bind_event_input_interface<T>`（:503-507）。
- `InputInterface<T>::activate()` 返回 `void*`（:114-117），不再暴露 `void**`。
- `ensure_registration_name_is_available()`（:469-496）：同一组件内输入/输出重名或种类冲突直接抛异常。
- 新增 **`EventInputInterface<T>`**（:125-155，构造即传回调、`register_input` 事件重载）、**`QueuedEventInputInterface<T>`**（:157-256：内部 `rmcs_utility::RingBuffer<T> queue_` + 独立 worker 线程 + 丢帧告警）、**`EventOutputInterface<T>`**（:302-378：`emit()`、监听者列表、`enable/disable/wait_idle` 原子状态机）。
- `register_input` / `register_output` 各两个重载（:382-432），`register_output` 增加 `requires std::constructible_from<T, Args...>`。
- `static std::string initializing_component_name;`（:445，v2 为 `const char*`）。
- `OutputInterface` 存储改为 `alignas(T) std::byte data_[sizeof(T)]`（:298），`activate()` 返回 `data_`（:284-288）。
- `Executor`（`executor.hpp:32-644`）：
  - 析构（:44-49）：`disable_event_outputs()` → `wait_event_outputs_idle()` → `thread_.join()`。
  - `add_component(const std::shared_ptr<Component>&)`（:51-57，改传引用）。
  - `start()`（:59-86）：`update_rate` 有限性与正数校验（:68-69）、周期可表示性校验（:70-74）；新增参数 `thread_config`（:77-79，`get_parameter_or<std::string>("thread_config", ..., "")` 经 `rmcs_utility::ThreadConfig{spec, "update"}` 解析）；`enable_event_outputs()`（:81）；线程入口 `thread_main(period, thread_config)`（:83-85）。
  - 统计（:89-126、:196-358）：`CumulativeStats`（`rmcs_utility::TDigest<double>` p50/p99、max、`update_count`、`skipped_count`，digest 尺寸 256）、`WindowStats`（每组件耗时和）、`ThreadStats`（每 5 s 上报）。
  - `thread_main()`（:128-...）：`thread_config.apply_to_current_thread()` 失败仅 WARN；循环体 `execute_update_iteration()` → `calculate_next_iteration_time()`（:207-222，落后时**整周期跳帧补偿**并累加 `skipped_count`）→ `log_due_reports()`；捕获 `std::exception` / `...` 后 `RCLCPP_FATAL` + `rclcpp::shutdown()`。
  - 时间戳语义：`predefined_msg_provider_->set_timestamp(scheduled_start)`（在 `execute_update_iteration` 内），并单独统计 `start_lateness_ms`。
  - `init()`（:437-614）：输出/输入**同时按 (name, kind) 建索引**；同名不同 `InterfaceKind` 视为致命冲突；错误信息统一为可读文本（`describe_output()` / `describe_declaration()`，:414-435）；配对调用 `input.bind(input.binding, output.binding)`（:576）；环形依赖报告含 `interface_kind_name`（:606-609）。配对顺序、日志文案（`Calculating component dependencies`、`- ` 缩进）与 v2 一致。
  - `append_updating_order()`（:616-633）与 v2 逻辑相同。
- `main.cpp`：`get_parameter("components", component_descriptions)` 不再检查返回值（:46）；`initializing_component_name = component_name`（:63，`std::string` 赋值）。**SIGSEGV handler 与其余部分与 v2 逐字节相同**（`diff -u` 仅 2 个 hunk）。
- 事件接口的实际使用者（v3 现状）：`rmcs_auto_aim_v2/src/component.cpp:51`、`rmcs_auto_aim_v2/src/io/capturer/capturer.cpp:624,651`、`.../imu_snapshot_buffer.hpp:133`、`.../recorder/player.cpp:342`、`recorder.cpp:324`、`video_player.cpp:155`。`rmcs_core` 目前**只用普通接口**。

## 3. 差异清单

### 3.1 逐文件差异（`diff -u` 实测）

| 文件 | diff 结论 | 关键差异 |
|---|---|---|
| `include/rmcs_executor/component.hpp` | 188 → 556 行（不减行，全部为新增/改写） | 见下 3.2 |
| `src/component.cpp` | 1 行 | `const char* Component::initializing_component_name` → `std::string Component::initializing_component_name` |
| `src/executor.hpp` | 186 → 646 行 | 见 3.3 |
| `src/main.cpp` | 2 个 hunk | `components` 缺失不再报错；`initializing_component_name` 赋值改为 `std::string` |
| `src/predefined_msg_provider.hpp` | 无差异 | 逐字节相同 |
| `README.md` | 无差异 | 逐字节相同 |
| `CMakeLists.txt` | 2 个 hunk | `-std=c++20`（v2，:6）→ `CMAKE_CXX_STANDARD 23` + `CMAKE_CXX_STANDARD_REQUIRED ON`（v3，:6-7） |
| `package.xml` | 1 行 | v3 新增 `<depend>rmcs_utility</depend>`（:14） |
| `plugins.xml` | 两侧都无 | 插件清单仍在 `rmcs_core/plugins.xml` 等包内，机制不变 |

### 3.2 `component.hpp` 逐条

1. **`before_pairing` 签名变化**：v2 `std::map<std::string, const std::type_info&>`（:26）→ v3 `OutputInfoMap`（`std::map<std::string, OutputInfo>`，:52/61）。
   - 影响：v2 侧唯一覆写者 `rmcs_core/src/broadcaster/value_broadcaster.cpp:22` 需改写（v3 已有等价实现 `rmcs_core/src/broadcaster/value_broadcaster.cpp:29`，且额外判断 `output.kind != InterfaceKind::Normal` → 必须保留该判断，否则事件接口会被当成普通 double 广播）。
   - 风险：任何移植 v2 的 `before_pairing` 代码若仍按 `const type_info&` 解引用（`type == typeid(double)`），将**编译失败**（`paired.second.type` 现在是 `reference_wrapper`，需 `output.type.get()`）。
2. **`register_input` / `register_output`（普通接口）签名基本兼容**：`(name, interface, required=true)` / `(name, interface, args...)` 保留；v3 追加 `requires std::constructible_from<T, Args...>` 约束与重名校验。
   - 风险：v3 新增的 `ensure_registration_name_is_available()` 会让"同一组件内 input 与 output 同名"或"N 次注册同名"**在运行期构造阶段抛异常**；v2 不检查。若 v2 组件存在这种（隐蔽的）写法，移植后会启动即崩。
3. **新增事件接口三件套**：`EventInputInterface<T>` / `QueuedEventInputInterface<T>` / `EventOutputInterface<T>`。v2 完全没有；`QueuedEventInputInterface` 依赖 `rmcs_utility::RingBuffer` 的 `push_back` / `peek_front` / `pop_front` / `readable()`（见主题 16）。
4. **`void**` → `void*` + `BindFunction` 间接层**：v2 `InputDeclaration::pointer_to_data_pointer`（:169）→ v3 `binding` + `bind`（:531-532）。这是内部实现差异，**对外 API 不变**；但任何访问 `Component` 私有成员的友元代码（如 v2 若有人写 `friend`）会失效——已确认 v2 下游无此类代码。
5. **`OutputInterface` 存储实现**：`std::aligned_storage_t`（v2:124）→ `alignas(T) std::byte data_[sizeof(T)]` + `std::launder`（v3:292-298）。`std::aligned_storage_t` 在 C++23 已 deprecated，v3 是必要修整。
6. **`initializing_component_name` 类型**：`const char*`（v2:156）→ `std::string`（v3:445）。`create_partner_component` 由 `name.c_str()` 改为直接赋值 `name`。下游无外部使用者（`grep` 确认仅 executor 包内使用），**无兼容性问题**；但注意 `initializing_component_name` 是全局静态，多线程创建组件仍不安全（两版一致）。
7. **`Component` 析构**：`virtual ~Component() {};` → `virtual ~Component() = default;`（语义不变）。
8. **新增 `InterfaceKind` / `EventState` 等公共符号**：会影响 `using namespace rmcs_executor;` 的命名空间污染范围（v2 下游如使用同名自定义 `EventState` 会冲突）——未发现实际冲突。

### 3.3 `executor.hpp` 逐条

1. **`update_rate` 参数校验**：v2 只判"取不到即抛"；v3 追加 `!std::isfinite || <= 0`、周期 `<= 0` 校验（:68-74）。影响：v2 的 YAML 若写了 `0` / 非法值，v2 会静默产生 `period` 异常行为，v3 直接启动失败（**更安全，但配置错误表现为启动崩溃**）。
2. **新增 `thread_config` 参数**（:77-79）：格式 `key=value;...`（`name` / `cpus` / `policy` / `priority` / `nice`）。v2 无任何线程配置能力；v3 组件 config 中目前**没有任何机器人配置该参数**（`grep thread_config rmcs_bringup/config` 只命中 executor 源码），默认空串 = no-op。
3. **调度策略**：v2 固定 `sleep_until(scheduled)` 且时间戳用计划点（`executor.hpp:61-66`）；v3 记录实际开始/结束、`start_lateness`，落后时按整周期跳帧（:207-222）并累加统计。
   - 影响：v2 在过载时会**逐步滑后**（累积相位漂移），v3 会**主动跳帧**保持节拍。对控制回路而言 v3 行为更接近"固定步长"，但单周期内某些组件会被跳过 → 依赖"每周期必调"的 v2 逻辑（如累积积分）需确认可接受。
   - 风险：`/predefined/update_count`（`predefined_msg_provider.hpp:17`）只统计**实际执行**次数，而 v3 统计日志里的 `Update` 含跳帧数；两者语义不同，排障时勿混淆。
4. **时间戳语义**：v2 `set_timestamp(next_iteration_time)`（未自增的计划值，等价于"本周期计划开始"）；v3 `set_timestamp(scheduled_start)`。语义一致，但 v3 是循环内显式 `scheduled_start`，v2 是循环外自增变量——**迁移后 `/predefined/timestamp` 的物理含义不变**。
5. **线程异常处理**：v2 循环体内抛异常 → 线程静默终止（`std::terminate` 与否取决于异常是否逃逸 `std::thread`，实际会终止进程）；v3 捕获后 `RCLCPP_FATAL` + `rclcpp::shutdown()`（:128-...）。
6. **新增统计上报**（每 5 s）：`Update/Skipped: a/b (c%)，Stat ms p50/p99/max: start_late ... update ...` 与 `Component 5s: name Xms/Y%, ...`（Top-3 组件）。依赖 `rmcs_utility::TDigest`。风险：日志量（每 5 s 2 行，可接受）；`std::format` 要求 C++20 及以上（v3 用 C++23）。
7. **事件输出的生命周期**：v3 析构先 `disable_event_outputs()` 再 `wait_event_outputs_idle()`（:44-49），`start()` 中 `enable_event_outputs()`（:81）。v2 无事件概念，无对应逻辑。
8. **`init()` 的配对模型**：v2 用 `unordered_map<string, OutputDeclaration*>`（按名字）；v3 用 `unordered_map<InterfaceKey, ...>`（name+kind）并额外维护 `output_by_name` / `declaration_kind_map`（:440-442、:448-521）。影响：v3 对"同名不同 kind"给出明确致命错误并给出可读描述；同时 v3 允许**同名不同 kind 的 output 共存**（例如普通 `/x` 与事件 `/x`）——若 v2 移植代码依赖"同名即重复"的旧假设，需注意。
9. **`append_updating_order` / 环形依赖检测**：逻辑与文案一致（v3 仅在依赖行追加 `interface_kind_name`）。
10. **参数加载方式**：executor 自身仍只用 `get_parameter`（`rclcpp::Node` + `automatically_declare_parameters_from_overrides(true)`），v3 的 `rmcs_utility::NodeMixin::param/param_or` 只被**组件**使用（如 `rmcs_core/src/debug/value_collector.cpp:25`），不影响 executor。

### 3.4 `main.cpp` 逐条

1. `components` 参数缺失：v2 抛异常（:46-47）→ v3 静默继续（`main.cpp:46`）。
   - 影响：v3 下配置路径写错 / YAML 丢失 → 启动成功但**零组件**（只有一个 update 线程 + predefined provider）。风险等级高（静默失败），建议迁移时**恢复该检查**。
2. `initializing_component_name`：`component_name.c_str()` → `component_name`（`std::string`，:63）。仅类型适配。
3. SIGSEGV handler（`segmentation_fault_handler`，:15-31）、`ClassLoader("rmcs_executor", "rmcs_executor::Component")`（:38-39）、正则解析（:49）、`executor->start()` / `spin()` / `shutdown()` 顺序（:68-71）：**与 v2 相同**。
4. 组件描述语法 `Plugin::Type -> instance_name` 与 v2 相同（见 `dual-infantry.yaml` 两侧一致）。

### 3.5 构建/包差异

1. `CMakeLists.txt:6` v2 `-std=c++20` → v3 `CMAKE_CXX_STANDARD 23` + `REQUIRED ON`。
   - 注意：`AGENTS.md` 第 6 节写"C++20"，与 v3 实际 `C++23`（executor/rmcs_utility 均如此）**不一致**，属文档待修正项。
   - 风险：v2 代码若使用 C++20 中含语义变化的构造（如 `std::aligned_storage_t`）在 C++23 下会告警/弃用；移植时按 v3 写法修整即可。
2. `package.xml` v3 新增 `rmcs_utility`（executor 现在直接 include `<rmcs_utility/ring_buffer.hpp>`、`tdigest.hpp`、`thread_config.hpp`）。v2 的 `rmcs_executor` 不依赖 `rmcs_utility`。
3. `plugins.xml` 机制无变化（仍在组件包内），但 **v3 的 executor 包同样没有 plugins.xml**——`pluginlib::ClassLoader` 的 `package` 参数仍是 `"rmcs_executor"`，因此 v3 的组件包必须导出 `<rmcs_executor plugin="${prefix}/plugins.xml"/>`（细节见主题 01/18，本主题仅记录 executor 侧不变）。

### 3.6 影响面汇总（v2 组件 → v3 executor）

| v2 用法 | v3 是否兼容 | 处理 |
|---|---|---|
| `register_input<T>(name, iface[, required])` | ✅ 源码兼容 | 无 |
| `register_output<T>(name, iface, args...)` | ✅（多 `constructible_from` 约束） | 若某组件用不可构造参数会编译失败，需 `未核实` 具体清单 |
| `before_pairing(旧签名)` | ❌ 编译失败 | 改 `OutputInfoMap` + `type.get()` |
| `create_partner_component<T>(name, args...)` | ✅ | 无（内部改用 `std::string`） |
| `InputInterface::make_and_bind_directly` / `bind_directly` | ✅ | 无 |
| 访问 `Component` 私有成员（`pointer_to_data_pointer` 等） | ❌ | 下游无使用者（`grep` 确认） |
| `Plugin::Type -> name` 配置语法 | ✅ | 无 |

## 4. 迁移方案（以 v3 为骨架）

**总原则：不改 v3 executor 架构，只补两处"回归"并在移植 v2 组件时按 3.6 适配。**

1. **保留 v3 全部新增能力**（事件接口、统计、跳帧、`thread_config`、异常处理、`init()` 的 kind 校验）——它们是 v3 的地基，且 `rmcs_auto_aim_v2` 已依赖事件接口（见 2. 末）。
2. **恢复 `components` 缺失的显式报错**（`rmcs_executor/src/main.cpp:46`）：
   ```cpp
   if (!executor->get_parameter("components", component_descriptions))
       throw std::runtime_error("Missing parameter 'components' or config is not found");
   ```
   理由：避免"配置路径写错 → 静默零组件启动"（主题 18 的 launch/config 变更会放大该风险）。
3. **移植 v2 组件时统一按 v3 API 适配**（不改 executor）：
   - `before_pairing(const OutputInfoMap&)` + `output.type.get()`；事件输出必须先判 `output.kind != InterfaceKind::Normal`（参考 v3 `value_broadcaster.cpp:29-36` 的既有正确写法）。
   - 新组件若需"同一名字既是普通输出又是事件输出"，按 v3 语义允许；但**不要**在同一组件内重复注册同名同 kind（会抛异常）。
   - 组件参数读取建议改用 `rmcs_utility::NodeMixin`（`node::param` / `param_or` / `info/warn/error`）以与 v3 组件风格一致（见主题 16）。
4. **移植 v2 硬件/设备代码时的 `RingBuffer` API 改名**：`front/back` → `peek_front/peek_back`，`writeable` → `writable`，`emplace_back_multi` → `emplace_back_n`，`pop_front_multi` → `pop_front_n`（v2 硬件模型此前用的是 librmcs 自带 `librmcs::utility::RingBuffer`，v3 统一为 `rmcs_utility::RingBuffer`）。详见主题 16。
5. **线程实时性**：若实机需要 `SCHED_FIFO` / CPU 绑定，直接在 v3 已有能力上通过 YAML 配置，不新增代码：
   ```yaml
   rmcs_executor:
     ros__parameters:
       update_rate: 1000.0
       thread_config: "name=rmcs-ctrl;cpus=2-3;policy=fifo;priority=80"
   ```
   （需容器 `--privileged` / 相应 capabilities；默认不改，保留 no-op。）
6. **日志级别**：每 5 s 的统计为 `RCLCPP_INFO`；如实机刷屏可后续加开关（**本阶段不改**，仅记录）。
7. **不做的事**：不回退 v3 的 C++23、不移除 `rmcs_utility` 依赖、不恢复 `void**` 绑定层、不删除事件接口。

## 5. 执行步骤 checklist

- [ ] 确认 v3 executor 为基线，不回退（03-0）
- [ ] `main.cpp`：恢复 `components` 缺失报错，并按 v3 风格保持 `std::string` 赋值（03-1）
- [ ] 移植 v2 组件时逐个核对 `before_pairing` 写法（`OutputInfoMap` + `type.get()` + `kind` 判断）（03-2，联动主题 05–13、17）
- [ ] 核对新移植组件是否存在"同组件内同名 input/output / 同名重复注册"（03-3）
- [ ] 移植 v2 硬件模型时同步替换 `RingBuffer` API 名称（03-4，联动主题 16、06、07）
- [ ] 全部移植组件改用 `NodeMixin` 读取参数（可选但建议，03-5）
- [ ] 交叉编译 arm64 全量通过（03-6）
- [ ] `nm -C` 核验新移植组件已注册为插件（03-7）
- [ ] 实机（至少 dual-infantry）观察 5 s 统计日志中 `Skipped` 比例，确认跳帧策略可接受（03-8）
- [ ] 决策：`thread_config` 是否对 dual-infantry 启用 FIFO/绑核（默认不改）（03-9）

## 6. 验收与验证

- 编译：`./.script/build-rmcs-cross --target-arch=arm64 --packages-skip rmcs_auto_aim_v2 --executor sequential`；日志写 `/tmp/opencode/` 或 `rmcs_ws/log-cross-arm64/`。
- 格式/静态：仓库根 `clang-format --dry-run -Werror`；单文件 `-fsyntax-only` 复用 `rmcs_ws/build-cross-arm64/rmcs_core/compile_commands.json`。
- 插件核验：`nm -C rmcs_ws/install-cross-arm64/lib/librmcs_core.so | grep registerPlugin<rmcs_core::hardware::DualInfantry>`。
- 运行期（离线不做）：日志中应出现 `Calculating component dependencies` + 缩进组件顺序；每 5 s 出现 `Update/Skipped: ...` 与 `Component 5s: ...`；`Skipped` 长期为 0 表示未过载。
- 负向验证：故意删掉 config 中的 `components`，应看到 `Missing parameter 'components' or config is not found`（若采纳 03-1）。

## 7. 备注与未决问题

- `AGENTS.md` 写"C++20"，而 `rmcs_executor/CMakeLists.txt:6` 与 `rmcs_utility/CMakeLists.txt:6` 实际为 C++23 —— 文档与实现不一致，建议在主题 02 统一处理。
- v3 `main.cpp` 去掉 `components` 检查属**上游行为**，恢复该检查等于与上游产生小差异；本文件建议恢复，最终需用户确认（未决）。
- `register_output` 的 `requires std::constructible_from<T, Args...>` 约束是否会让某些 v2 组件写法编译失败：尚未逐组件核验（**未核实**），需在移植各主题时逐个编译验证。
- v3 `QueuedEventInputInterface` 的 worker 线程异常会**直接退出 worker**（`component.hpp:225-236` 中 `return`），事件将静默丢失（只打 ERROR 日志）——属 v3 现有行为，本阶段不改，仅记录。
- v3 统计使用 `std::format`（`executor.hpp:320,348,351`）与 `TDigest`，交叉编译工具链需满足 libstdc++ 对 `std::format` 的支持（当前 arm64 交叉编译已通过，说明满足）。
- 本主题不涉及 v2 的 `autopilot_client`（不在迁移范围），但其 `plugins.xml` 与 executor 用法可作为 API 兼容性参考样本。
