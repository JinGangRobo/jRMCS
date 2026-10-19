# 05 硬件栈：librmcs 集成方式与设备驱动逐文件差异

- 状态：待办
- 优先级：P1（dual-infantry 依赖，故实际阻塞 P0）
- 影响机型：dual_infantry（P0）、dual_sentry、hero、mini_infantry；其它机型（legacy）一并受影响
- 依赖主题：06 硬件模型-dual_infantry、07 硬件模型-其余机型
- 验收方式：v3 侧 `npm-style` 对照表逐条核对；对每个 device 头文件做 `diff -u`；确认 v3 构建通过（`librmcs_core.so` 可加载、插件符号存在）

> 本文所有结论均来自对真实文件的读取与 `diff -u`。路径与行号均可追溯；不确定项已显式标注「未核实」。

## 1. v2（reATRM）现状

### 1.1 集成方式：内嵌 librmcs 头文件 + `-lusb-1.0`

- 内嵌子模块（仅头文件）：`/workspaces/reATRM/src/core/utils/system_ctrl/rmcs_core/librmcs/librmcs/`
  - `client/cboard.hpp`
  - `device/bmi088.hpp`、`device/buzzer.hpp`、`device/dji_motor.hpp`、`device/dji_motor_with_encoder.hpp`、`device/dm_motor.hpp`、`device/dr16.hpp`、`device/lk_motor.hpp`
  - `utility/cross_os.hpp`、`utility/ekf_imu.hpp`、`utility/endian_promise.hpp`、`utility/logging.hpp`、`utility/pid_calculator.hpp`、`utility/ring_buffer.hpp`
  - 注意：`dji_motor_with_encoder.hpp` **不在** `rmcs_core/src/hardware/device/` 下，而在内嵌 librmcs 的 `device/` 下，且 v2 没有任何 rmcs_core 包装类引用它（`grep` 无命中）。任务描述中把它归到 `hardware/device/` 是路径笔误。
- CMake：`/workspaces/reATRM/src/core/utils/system_ctrl/rmcs_core/CMakeLists.txt`
  - `set(CMAKE_CXX_FLAGS "... -std=c++20")`（第 5 行）
  - `include_directories(${PROJECT_SOURCE_DIR}/librmcs)`（第 25 行）
  - `include_directories(SYSTEM "/usr/include/libusb-1.0")`（第 27 行）
  - `target_link_libraries(${PROJECT_NAME} -lusb-1.0)`（第 29 行）
  - 无外部 SDK 下载、无 FetchContent。

### 1.2 板卡传输层：`librmcs::client::CBoard`

`librmcs/librmcs/client/cboard.hpp`（共 821 行）要点：

- `class CBoard`，`explicit CBoard(int32_t usb_pid = -1)`（第 22 行）→ `init(0x0d00, usb_pid)`（第 23 行）：**vendor id 固定 0x0d00，product id 由参数给出**，即上层用 `usb_pid` 区分 top/bottom 板。
- 自行管理 libusb：`libusb_init` / `libusb_get_device_list` / `libusb_open` / `libusb_claim_interface` / `libusb_alloc_transfer`（第 115–234 行）；`target_interface = 0x01`、`out_endpoint = 0x01`、`in_endpoint = 0x81`、接收缓冲 64 字节（第 508–517 行）。
- 事件线程由使用方手写：`void handle_events()`（第 35 行）/ `stop_handling_events()`（第 62 行），典型用法为在板对象里 `std::thread event_thread_([this](){ handle_events(); });`。
- 上行字段解析：`UpwardId` 枚举 `CONTROL=0, GPIO=1, CAN1=2, CAN2=3, CAN3=4, UART1=5..UART6=10, IMU=11`（第 397–414 行）；`DownwardId` 多出 `LED=11, BUZZER=12`（第 416–434 行）。
- 回调（virtual，第 65–110 行）：`can1_receive_callback(uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission, uint8_t len)`、`can2_receive_callback(...)`、`uart1_receive_callback(const std::byte*, uint8_t)`、`uart2_receive_callback(...)`、`dbus_receive_callback(...)`、`accelerometer_receive_callback(int16_t x,y,z)`、`gyroscope_receive_callback(int16_t x,y,z)`、`putter_receive_callback(bool)`、`camera_capturer_callback(bool)`。
- 下行：`class CBoard::TransmitBuffer`（第 524 行）提供 `add_can1_transmission(can_id, can_data, ...)`、`add_can2_transmission(...)`、`add_uart1_transmission(...)`、`add_uart2_transmission(...)`、`add_dbus_transmission(...)`、`add_buzzer_transmission(uint8_t)`、`trigger_transmission()`（第 591–629 行）；每个 transfer 缓冲 64 字节，需手动 `trigger_transmission()` 提交。

### 1.3 `hardware/device/*.hpp` 的形态：ROS 装饰器（继承 librmcs 设备类）

v2 的 `device/` 头文件普遍是「薄装饰器」：继承 `librmcs::device::Xxx`，在构造函数里 `register_output/register_input` 把 ROS 接口挂上，并把 librmcs 的回调/命令转成 `uint64_t can_data` / `uint16_t`。

| 文件 | 行数 | 类 | 关系 | 注册接口（摘要） |
|---|---|---|---|---|
| `benewake.hpp` | 87 | `Benewake` | 独立（无 librmcs 依赖） | 见 v3（两版**逐字节相同**） |
| `bmi088.hpp` | 8 | — | 仅 `using Bmi088 = librmcs::device::Bmi088;` | **不注册**（由 model 直接使用 librmcs 类） |
| `buzzer.hpp` | 40 | `Buzzer : librmcs::device::Buzzer` | 装饰器 | input `/buzzer/score`（`std::optional<BuzzerScore>`）；`update_status()`/`generate_command()` 转发 |
| `dji_motor.hpp` | 96 | `DjiMotor : librmcs::device::DjiMotor` | 装饰器 | outputs `…/angle,raw_angle,velocity,torque,max_torque,velocity_filtered,alive`；input `…/control_torque` |
| `dm_motor.hpp` | 109 | `DmMotor : librmcs::device::DmMotor` | 装饰器 | outputs `…/angle,raw_angle,velocity,velocity_filtered,torque,max_torque,alive`；inputs `…/control_torque,control_velocity` |
| `dr16.hpp` | 125 | `Dr16 : librmcs::device::Dr16` | 装饰器 | outputs `/remote/joystick/{right,left}`,`/remote/switch/{right,left}`,`/remote/mouse/velocity`,`/remote/mouse/mouse_wheel`,`/remote/mouse`,`/remote/keyboard`,`/remote/rotary_knob`,`/remote/rotary_knob_switch` |
| `gy614.hpp` | 80 | `Gy614` | 独立（自解析串口） | output `/friction_wheels/temperature`（由构造函数传入前缀） |
| `hipnuc.hpp` | 109 | `Hipnuc` | 独立 | **v2 非 legacy 代码未引用** |
| `lk_motor.hpp` | 163 | `LkMotor : librmcs::device::LkMotor` | 装饰器 | outputs `…/angle,velocity,torque,temperature,max_torque,alive`；inputs `…/control_torque,control_velocity,control_angle,control_angle_shift` |
| `supercap.hpp` | 85 | `Supercap` | 独立 | outputs `/chassis/power`,`/chassis/supercap/voltage`,`/chassis/supercap/energy_percentage`,`/chassis/supercap/enabled` |

- 设备驱动的**协议/换算逻辑**在 librmcs 里：例如 `librmcs/device/dji_motor.hpp:58-106` 的换算系数、`librmcs/device/dm_motor.hpp:65-124` 的 MIT 协议与误差枚举、`librmcs/device/dr16.hpp:16-80` 的 DBUS 位域解析、`librmcs/utility/ekf_imu.hpp`（558 行，自带矩阵库的 AHRS/EKF）与 `librmcs/device/bmi088.hpp:13-80`（构造参数 `(process_noise1, process_noise2, measurement_noise, q0..q3)` + `set_coordinate_mapping`）。
- v2 `librmcs/device/buzzer.hpp` 提供蜂鸣器乐谱协议（`Tone{OFF,LOW,MEDIUM,HIGH}`、`Mode{CONTINUOUS,ONCE}`、`BuzzerScore`、`generate_command()`）。

### 1.4 v2 插件注册（`plugins.xml`）

- 文件：`/workspaces/reATRM/src/core/utils/system_ctrl/rmcs_core/plugins.xml`，共 42 条 `class type`。
- 硬件相关：`hardware::cboard::Status`、`hardware::PidTuneTool`、`hardware::DualSentry`、`hardware::DualInfantry`、`hardware::MiniInfantry`、`hardware::Hero`、`hardware::SteeringHero`、`hardware::SteeringInfantry`、`hardware::TunnelInfantry`、`hardware::TestingInfantry`，以及 `hardware::cboard::Command`、`hardware::referee::Status`、`hardware::cboard::GyroCalibrator`。
- **已核实**：`hardware::cboard::*`、`hardware::referee::Status`、`TestingInfantry`、`SteeringHero`、`SteeringInfantry`、`TunnelInfantry` 在 v2 源码树中**找不到对应实现**（`grep -rn` 无命中、无同名文件），属于历史遗留的失效注册项。

## 2. v3（jRMCS）现状

### 2.1 集成方式：FetchContent 外部 SDK `librmcs-sdk` + `BUILD_STATIC_LIBRMCS`

- CMake：`/workspaces/RMCS/rmcs_ws/src/rmcs_core/CMakeLists.txt`
  - `set(CMAKE_CXX_STANDARD 23)`（第 6 行）——v2 是 C++20。
  - `include(FetchContent)` → `set(BUILD_STATIC_LIBRMCS ON CACHE BOOL "Build static librmcs SDK" FORCE)` → `FetchContent_Declare(librmcs URL https://github.com/JinGangRobo/libjrmcs/releases/download/v3.3.0a7/librmcs-sdk-src-3.3.0-alpha.7-debug.zip URL_HASH SHA256=0d28bc564f361bd7a7bfb8b25226de813c7f71d5d180b583c4888b750bd39379 DOWNLOAD_EXTRACT_TIMESTAMP TRUE)` → `FetchContent_MakeAvailable(librmcs)`。
  - `target_link_libraries(${PROJECT_NAME} librmcs-sdk)`；**不再链接 `-lusb-1.0`**，也不再 `include_directories(librmcs)`（libusb 由 SDK 内部处理）。
- SDK 实际落盘（已解压，可核对）：`/workspaces/RMCS/rmcs_ws/build-cross-arm64/rmcs_core/_deps/librmcs-src/`
  - `core/include/librmcs/data/datas.hpp`；`core/include/librmcs/spec/{can,uart,gpio}.hpp` + `spec/{c_board,rmcs_board_lite,rmcs_board_pro}/{can,uart,gpio}.hpp`
  - `host/include/librmcs/board/{c_board,common,rmcs_board_lite,rmcs_board_pro}.hpp`、`host/include/librmcs/protocol/handler.hpp`
  - 静态库：`build-cross-arm64/rmcs_core/_deps/librmcs-build/host/liblibrmcs-sdk.a`（安装到 `install-cross-arm64/lib/liblibrmcs-sdk.a`）
  - 已解压源码还保留了 `librmcs-sdk-src-3.3.0-alpha.7-debug.zip`。
- **关键结论**：v3 SDK **不再提供 `librmcs::device::*` 设备驱动**（`core/include` 只有 `data/`、`spec/`）。所以 v2 的 `librmcs/device/*.hpp` 被**下沉/重写进 `rmcs_core/src/hardware/device/`**，成为不依赖 librmcs 的独立驱动。

### 2.2 板卡传输层：`librmcs::board::CBoard` + `Callback`

`_deps/librmcs-src/host/include/librmcs/board/c_board.hpp` 要点：

- `class CBoard final`（第 36 行）；构造：`explicit CBoard(Callback& callback = default_callback_, std::string_view serial_filter = {}, const AdvancedOptions& options = {})` → `handler_(0x0D00, 0xD401, serial_filter, options, callback)`（第 124–127 行）。
  - **vendor id 0x0D00、product id 0xD401 硬编码**，改用 **serial 字符串过滤** 选择板卡（对应 `board_serial_top_board/bottom_board`）。
- 内部线程由 SDK 管理（`host::protocol::Handler`），**上层不再手写 `event_thread_`/`stop_handling_events()`**。
- 回调基类 `CBoard::Callback : public data::DataCallback`（第 38 行），暴露嵌套 `Spec` / `View`：
  - `Spec::kCans`（`spec::c_board::internal::CanDescriptors`）、`Spec::kUarts`、`Spec::kGpios`。
  - 回调：`can_receive_callback(const Spec::Can&, const View::Can&)`、`uart_receive_callback(const Spec::Uart&, const View::Uart&)`、`gpio_digital_read_result_callback(...)`、`gpio_analog_read_result_callback(...)`，以及 `accelerometer_receive_callback(View::ImuAccelerometer)`、`gyroscope_receive_callback(View::ImuGyroscope)`、`temperature_receive_callback(View::ImuTemperature)`（第 63–88 行）。IMU 视图带时间戳 `timestamp_quarter_us`。
- C 板 spec（`core/include/librmcs/spec/c_board/*.hpp`）：
  - CAN 只有 `kCan1`、`kCan2`（`can.hpp:60-61`）——v2 的 CAN3 消失，且不再有「CAN1/CAN2 各自独立回调」，改为单一 `can_receive_callback` + `can` 描述符判断。
  - UART 为 `kDbus`、`kUart1`、`kUart2`（`uart.hpp:61-63`）——v2 的 UART1..UART6 与 `dbus`(=UART3) 语义被规整为 `kDbus/kUart1/kUart2`。
- 下行：`CBoard::PacketBuilder`（第 135 行）链式 `can_transmit(Spec::kCans.kCanX, data)`、`uart_transmit(...)`、`uart_config(...)`、`gpio_*`；由 `board_->start_transmit()` 取得，**无需显式 trigger**（生命周期结束即提交）。数据结构用 `librmcs::data::*View`（如 `CanDataView{.can_id, .can_data}`）。

### 2.3 `hardware/device/` 的形态：独立驱动（不注册 ROS，不继承 librmcs）

| 文件 | 行数 | 类 | 备注 |
|---|---|---|---|
| `benewake.hpp` | 87 | `Benewake` | 与 v2 **完全相同**；当前 v3 非 legacy 代码未引用 |
| `bmi088.hpp` | 169 | `Bmi088` | **重写**为独立 Mahony AHRS 驱动（`Bmi088(sample_freq, kp, ki, q0..q3)` + `set_coordinate_mapping` + `store_accelerometer_status/store_gyroscope_status/update_status`）；当前 4 个机型均未使用（用的是 `bmi088_ekf`） |
| `bmi088_ekf.hpp` | 306 | `Bmi088Ekf` | **新增**；`Config{Eigen::Matrix3d body_to_sensor}`，接口 `push_accelerometer_sample(x,y,z,time)`、`try_update_with_gyroscope_sample(x,y,z,time)->optional<Snapshot>`、`snapshot()->optional<ImuSnapshot>`；依赖 `filter/imu_ekf.hpp`、`rmcs_msgs/{board_clock,imu_snapshot}.hpp` |
| `board_clock_lifter.hpp` | 53 | `BoardClockLifter` | **新增**；`advance_timebase(raw_quarter_us)`、`timebase()`、`lift_timestamp(raw_quarter_us)`，把下位机 32 位时间戳提升为 `rmcs_msgs::BoardClock::time_point` |
| `can_packet.hpp` | 67 | `ByteConvertible<T>`、`CanPacket8`（`Quarter`/`PaddingQuarter`） | **新增**；统一 8 字节 CAN 包与 `std::span<const std::byte>` 互转（`as_bytes()`）；`CanPacket8{Quarter×4}` 便于组包 |
| `dji_motor.hpp` | 315 | `DjiMotor` | **重写**（详见差异清单） |
| `dm_motor.hpp` | 400 | `DmMotor` | **重写** |
| `dr16.hpp` | 286 | `Dr16` | **重写**（不再注册接口；新增有效性与超时） |
| `gy614.hpp` | 80 | `Gy614` | 与 v2 **完全相同**；dual_sentry 仍用 |
| `hipnuc.hpp` | 109 | `Hipnuc` | 与 v2 **完全相同**；未引用 |
| `j_supercap.hpp` | 86 | `Jsupercap` | **新增**：等价于 v2 的 `supercap.hpp`（能量百分比算法），但 `store_status(std::span)`；dual_sentry/hero/mini 使用 |
| `lk_motor.hpp` | 565 | `LkMotor` | **重写** |
| `remote_control.hpp` | 177 | `RemoteControl` | **新增**：dr16/vt13 仲裁，注册全部 `/remote/*` 输出；`register_dr16()`/`register_vt13()`/`update()` |
| `supercap.hpp` | 118 | `Supercap` | **新增的另一套协议**（`Supercap(status,command)`，输出 `chassis/voltage`，读 `/referee/chassis/output_status`、`/chassis/supercap/charge_power_limit`，`generate_command()`/`generate_disable_command()`）；当前未被机型使用 |
| `vt13.hpp` | 344 | `Vt13` | **新增**：VT13 遥控 + 裁判帧（`ModeSwitch`、`ReadResult=variant<Incomplete,VerificationFailed,Success>`、`RefereeFrameHeader`）；仅被 `remote_control.hpp` 引用 |

### 2.4 v3 插件注册（`plugins.xml`）

- 文件：`/workspaces/RMCS/rmcs_ws/src/rmcs_core/plugins.xml`，共 58 条。
- 硬件：`hardware::DualInfantry/DualSentry/Hero/MiniInfantry`、`hardware::Px4VisionBridge`。
- 已清理 v2 的失效注册（`cboard::*`、`hardware::referee::Status`、`TestingInfantry` 等）。
- 新增控制器/组件：`SmcDualYawController`、`JChassisPowerController`、`JOmniWheelController`、`HeroGimbalController`、`HeroChassisController`、`HeroFrictionWheelController`、`EccentricDualYaw`、`PutterController` 等。
- 注意（已核实）：`plugins.xml` 中 `rmcs_core::controller::chassis::JOmniWheelController` **重复注册两次**（第 13 行与第 24 行），虽不致失败，但属冗余，建议清理。

## 3. 差异清单

### 3.1 集成层

| # | 差异点 | v2 做法 | v3 做法 | 影响 | 风险 |
|---|---|---|---|---|---|
| A1 | 依赖来源 | 内嵌头文件子模块 `rmcs_core/librmcs/librmcs/` | FetchContent 外部 SDK `librmcs-sdk` v3.3.0-alpha.7（URL+SHA256） | 构建需要联网下载或缓存；版本升级与仓库解耦 | 中：离线环境必须先有 SDK 缓存（`_deps` 已存在，可复用） |
| A2 | 编译标准 | C++20 | C++23 | 新驱动大量使用 `std::span`、`std::unreachable`、`requires`、designated init、`std::bit_cast` 等 | 低-中：v2 侧代码不宜直接回移 |
| A3 | 设备驱动位置 | SDK 提供 `librmcs::device::*`，rmcs_core 只写装饰器 | SDK 不再提供 device，驱动**下沉**到 `rmcs_core/src/hardware/device/` | 驱动成为本仓可维护资产 | 中：需保证协议换算与 v2 数值一致 |
| A4 | libusb | 直接 `#include <libusb.h>` + `-lusb-1.0` + 手写传输 | SDK 内部封装（静态库 `liblibrmcs-sdk.a`） | 上层不再管理 USB；无需 udev/libusb 头 | 低 |
| A5 | 板卡识别 | `CBoard(int32_t usb_pid)`，VID 0x0d00 + PID（top `0x9d15` / bottom `0x34a4`） | `CBoard(callback, serial_filter)`，VID 0x0D00 + **PID 固定 0xD401** + serial 字符串 | 必须获取并填写两块板的 serial；PID 不再是区分手段 | **高**：serial 填错将选不到板卡而抛异常 |
| A6 | 事件线程 | 用户手写 `std::thread` + `handle_events()/stop_handling_events()` | SDK 内部线程 | 上层析构逻辑简化（`~TopBoard() = default`） | 低 |
| A7 | 下行组包 | `TransmitBuffer::add_can1/2_transmission(...)` + `trigger_transmission()` | `board_->start_transmit().can_transmit(Spec::kCans.kCanX, {...})` 链式 | 组包代码需整体改写 | 中 |
| A8 | 上行回调 | `can1_receive_callback`/`can2_receive_callback`/`uart1`/`uart2`/`dbus` + `(can_id, uint64_t can_data, ...)` | 统一 `can_receive_callback(const Spec::Can&, const View::Can&)` / `uart_receive_callback(const Spec::Uart&, const View::Uart&)`；数据为 `span` | CAN/UART 分发逻辑重写 | 中 |
| A9 | 总线集合 | CAN1/CAN2/CAN3；UART1..UART6（dbus=UART3） | CAN1/CAN2；UART = kDbus/kUart1/kUart2 | 使用 CAN3 或 UART4-6/dbus 以外通道的机型需改接线或驱动 | **高**（对有此类接线的机型；本仓 4 机型未用到 CAN3） |
| A10 | IMU 数据 | `accelerometer_receive_callback(int16_t,int16_t,int16_t)`，无时间戳 | 视图带 `timestamp_quarter_us`，需配 `BoardClockLifter` | 姿态估计改为带时间戳的 EKF | 中 |
| A11 | 蜂鸣器 | `add_buzzer_transmission()` + `librmcs::device::Buzzer` | SDK 与 rmcs_core **均无** buzzer | v3 全部机型失去蜂鸣器（含裁判系统的提示音） | **高**（功能缺失，非编译问题） |
| A12 | GPIO(putter/camera) | `putter_receive_callback`/`camera_capturer_callback` | 改为 `gpio_digital_read_result_callback`/`gpio_analog_*` + `gpio_digital_write/read` | 依赖 GPIO 的机型需迁移到新 GPIO 抽象 | 中 |

### 3.2 设备驱动逐文件（两两 `diff -u`）

| # | 文件 | v2 | v3 | 差异性质 | 接口差异要点 |
|---|---|---|---|---|---|
| D1 | `benewake.hpp` | 87 行 | 87 行 | **完全一致** | 无 |
| D2 | `gy614.hpp` | 80 行 | 80 行 | **完全一致** | 无（`Gy614` 输出 `/friction_wheels/temperature`） |
| D3 | `hipnuc.hpp` | 109 行 | 109 行 | **完全一致** | 无（两版均未被非 legacy 引用） |
| D4 | `bmi088.hpp` | 8 行，`using Bmi088 = librmcs::device::Bmi088;` | 169 行，独立 `class Bmi088` | **重写** | v2 构造 `(process_noise1, process_noise2, measurement_noise, q0..q3)` + librmcs EKF；v3 构造 `(sample_freq, kp, ki, q0..q3)` 的 Mahony AHRS。v3 该头文件**未被 4 机型使用** |
| D5 | `bmi088_ekf.hpp` | 不存在 | 306 行，`Bmi088Ekf` | **新增** | `Config{body_to_sensor}`；`push_accelerometer_sample/try_update_with_gyroscope_sample/snapshot`。取代 v2 librmcs 的 `librmcs::device::Bmi088`+`utility/ekf_imu.hpp` |
| D6 | `board_clock_lifter.hpp` | 不存在 | 53 行 | **新增** | 时间戳提升（32 位→64 位 board clock） |
| D7 | `can_packet.hpp` | 不存在 | 67 行 | **新增** | `CanPacket8`/`Quarter`/`PaddingQuarter`/`ByteConvertible`，`span` 互转 |
| D8 | `buzzer.hpp` | 40 行，`Buzzer : librmcs::device::Buzzer` | **删除** | **删除** | v2 output `/buzzer/score`（`std::optional<BuzzerScore>`）+ `generate_command()`；v3 无对应能力 |
| D9 | `dji_motor.hpp` | 96 行，`DjiMotor : librmcs::device::DjiMotor` | 315 行，独立 `DjiMotor` | **重写** | 见 D9 详表 |
| D10 | `dji_motor_with_encoder.hpp` | 仅在内嵌 librmcs `librmcs/device/` 下（`Type{GM6020,GM6020_VOLTAGE,M3508,M2006}`、`generate_command(double)->uint16_t`） | 不存在 | **删除** | v2 rmcs_core 未使用；v3 用 `DjiMotor` 统一处理 |
| D11 | `dm_motor.hpp` | 109 行，`DmMotor : librmcs::device::DmMotor` | 400 行，独立 `DmMotor` | **重写** | 见 D11 详表 |
| D12 | `dr16.hpp` | 125 行，`Dr16 : librmcs::device::Dr16` | 286 行，独立 `Dr16` | **重写** | 见 D12 详表 |
| D13 | `lk_motor.hpp` | 163 行，`LkMotor : librmcs::device::LkMotor`（Type 4 种） | 565 行，独立 `LkMotor`（Type 6 种） | **重写** | `Type` 由 `MG5010E_I10/MG4010E_I10/MG6012E_I8/MG4005E_I10` → `kMG5010Ei10/kMG4010Ei10/kMG6012Ei8/kMG4005Ei10/kMG5010Ei36/kMHF7015`；`store_status(uint64_t)`→`store_status(std::span)`；新增 output `…/raw_angle`；`generate_*_command()` 返回 `CanPacket8`；新增状态报文过滤（`command & 0xF0 != 0x80` 丢弃） |
| D14 | `supercap.hpp` | 85 行（`Supercap(status, max_capacity_voltage=23.0)`） | 118 行（**另一套协议**） | **语义替换** | v2：输出 `/chassis/power`、`/chassis/supercap/{voltage,energy_percentage,enabled}`；`store_status(uint64_t)`。v3：输出 `/chassis/power`、`/chassis/voltage`、`/chassis/supercap/{voltage,enabled}`（**去掉 energy_percentage**，**新增 chassis/voltage**），新增 input `/referee/chassis/output_status`、`/chassis/supercap/charge_power_limit`，新增 `generate_command()`/`generate_disable_command()`（下发启停与充电限功率）。当前 v3 未使用 |
| D15 | `j_supercap.hpp` | 不存在（等价逻辑在 v2 `supercap.hpp`） | 86 行，`Jsupercap` | **新增（等价迁移）** | 与 v2 `supercap.hpp` 同输出（含 `energy_percentage`），仅 `store_status(std::span)`；`constructor(status_component, max_capacity_voltage=23.0)`。dual_sentry/hero/mini 在用 |
| D16 | `remote_control.hpp` | 不存在 | 177 行，`RemoteControl` | **新增** | 注册 `/remote/*` 全部输出；`register_dr16/register_vt13` + 超时互锁 + 数据源仲裁（VT13 Sport / VT13 Cine 安全 / DR16 / 空安全态） |
| D17 | `vt13.hpp` | 不存在 | 344 行，`Vt13` | **新增** | VT13 遥控 + 裁判帧解析，`ModeSwitch`、`ReadResult` |

#### D9 详表：`dji_motor.hpp`

| 项 | v2 | v3 |
|---|---|---|
| 基类 | `: public librmcs::device::DjiMotor` | 无（独立） |
| `Config` | `Config(Type)`，无 id；`enum Type{GM6020, GM6020_VOLTAGE, M3508, M2006}` | `Config(Type, id=0)`；`enum class Type{kGM6020, kGM6020Voltage, kM3508, kM2006}` |
| 输出接口 | `…/angle, raw_angle, velocity, torque, max_torque, velocity_filtered, alive` | `…/angle, raw_angle, velocity, torque, max_torque, alive`（**`velocity_filtered` 取消**） |
| 输入接口 | `…/control_torque` | 同 |
| 状态写入 | `store_status(uint64_t can_data)` | `store_status(std::span<const std::byte>)`；新增 `match_then_store_status(can_id, span)`、`recv_id()/send_id()/id()` |
| 下发 | `generate_command()->uint16_t` | `generate_command()->CanPacket8::Quarter`；新增 `operator<<(CanPacket8&, const DjiMotor&)` |
| 换算 | 复用 librmcs：`torque_constant/raw_current_max/current_max` 见 `librmcs/device/dji_motor.hpp:73-101` | 内联同值（`kGM6020:0.741/16384/3`，`kGM6020Voltage:0.741/25000/3`，`kM3508:0.3*187/3591/16384/20`，`kM2006:0.18/36/16384/10`） |
| 其他 | — | 新增 `calibrate_zero_point()`、`temperature()`；删除成员独立 `velocity_lpf_`（DM 仍保留 `velocity_filtered`） |

> 注意：v3 `DjiMotor` 的 `kGM6020`/`kGM6020Voltage` 对应「电流环 0x1FE」/「电压环 0x1FF」两种发送帧（`send_id()`），v2 的 `GM6020`/`GM6020_VOLTAGE` 命名相同；但 v2 model 里 top/bottom yaw 使用 `GM6020`（0x1FE 语义）却把命令发到 0x1FF，v3 改用 `kGM6020Voltage` 并把命令发到 0x1FF（详见主题 06 的 M5）。

#### D11 详表：`dm_motor.hpp`

| 项 | v2 | v3 |
|---|---|---|
| 基类 | `: public librmcs::device::DmMotor` | 无 |
| 类型 | `enum Type{J4310}` | `enum class Type{kJ4310}` |
| 错误枚举 | `DmMotorErrorMsg{DISABLE=0, ENABLE=1, OVER_VOLTAGE=8, UNDER_VOLTAGE=0, OVER_CURRENT=11, MOS_OVER_TEMPERATURE=12, ROTOR_OVER_TEMPERATURE=13, COMMUNICATION_ERROR=0xd, OVERLOAD=15}` | `ErrorMsg{kDisable=0,kEnable=1,kOverVoltage=8,kUnderVoltage=9,kOverCurrent=10,kMosOverTemperature=11,kRotorOverTemperature=12,kCommunicationError=13,kOverload=14}`（**修正**，原表多项错位） |
| 输出接口 | `…/angle, raw_angle, velocity, velocity_filtered, torque, max_torque, alive` | 增加 `…/temperature`、`…/error_code`（`velocity_filtered` 保留） |
| 状态写入 | `store_status(uint64_t)` | `store_status(std::span<const std::byte>)`（成员 `can_packet_`） |
| 下发 | `generate_torque_command()->uint64_t`、`generate_velocity_command()->uint64_t` | `generate_torque_command()->CanPacket8`、`generate_velocity_command()->CanPacket8`；故障时 `generate_error_command(0xfc/0xfb)` |
| 反转处理 | 位置/指令按 `reversed_` 翻转，**速度未翻转** | 速度按 `reversed_` 翻转（源码注释给出实测 corr=-0.98 的证据）；`max_torque_=10.0`（J4310 文档值） |
| 其他 | — | 新增 `calibrate_zero_point()`、`mos_temperature()`、`last_error_msg()`；`velocity_filter_` 用 `filter/low_pass_filter.hpp` |

#### D12 详表：`dr16.hpp`

| 项 | v2 | v3 |
|---|---|---|
| 基类 | `: public librmcs::device::Dr16`（并把 librmcs 的 `Vector/Switch/Mouse/Keyboard` 转成 Eigen/`rmcs_msgs`） | 独立 `Dr16`（自带 `Vector`/`Switch`/`Mouse`/`Keyboard` 定义） |
| ROS 接口 | 构造函数内注册全部 `/remote/*`（含 `rotary_knob_switch`） | **不注册**；改由 `RemoteControl` 注册 |
| `update_status()` | 转发 librmcs 后写输出 | 自行解析并计算 `rotary_knob_switch`；新增**新鲜度**机制：`kFreshTimeout=500ms`、`valid()`、`set_timeout_enabled()`，超时清零 |
| 初始挡位 | `Switch::DOWN`（`data_part1_` 初值） | `Switch::kUnknown` |
| 其它 | `mouse_wheel_` 在 librmcs 内 | 同算法（`-part2.mouse_velocity_z/32768.0`） |

## 4. 迁移方案（以 v3 为骨架）

1. **保持 v3 骨架不变**，不为迁就 v2 而回退 C++20 或重新内嵌 librmcs。
2. **librmcs 集成**：维持 FetchContent + `BUILD_STATIC_LIBRMCS`；把 SDK URL/SHA256 纳入固定版本策略，并确认离线可复用 `_deps/librmcs-src` 缓存。
3. **设备驱动**：以 v3 `hardware/device/` 为唯一真源；对 v2 中 v3 缺失的能力按「补驱动」而非「改架构」处理：
   - **buzzer（P0 功能缺口）**：v2 有、v3 无。重写一个不依赖 librmcs 的 `device/buzzer.hpp`（可直接复用 v2 `librmcs/device/buzzer.hpp` 的协议：`Tone/Mode/BuzzerScore/generate_command()`），并在 SDK 的 `PacketBuilder` 中补一条下行通道（当前 SDK C 板 spec 无 buzzer 字段，**需要下位机协议支持**，标为「未核实」）。备选：若下位机已不支持蜂鸣器，则在 v3 明确记录「蜂鸣器能力取消」。
   - **supercap**：明确二选一——`j_supercap.hpp`（沿用 v2 老协议，继续在 dual_sentry/hero/mini 使用）与 `supercap.hpp`（新协议，含充电限功率闭环）。dual_infantry 需决定采用哪一种（主题 06 详述）。
   - **GPIO（putter/camera）**：迁移到 `gpio_digital_read/write` + `gpio_analog_*`；使用方在 legacy 机型中，非本阶段重点。
4. **接口命名对齐**：`velocity_filtered` 在 `DjiMotor` 已取消 → 任何消费 `/…/velocity_filtered` 的配置（如 v2 的 `bullet_feeder_velocity_pid_controller`）必须改为 `/…/velocity` 或改用 `DmMotor`。
5. **配置与插件**：`usb_pid_*` → `board_serial_*` 全量替换；`plugins.xml` 清理重复注册。
6. **legacy 处理**：v2 legacy 有 4 个 v3 没有的模型（`infantry`、`steering-hero`、`steering-infantry`、`tunnel_infantry`），v3 legacy 有 6 个 v2 没有的。是否补充属于「按需」，不阻塞 P0。

## 5. 执行步骤 checklist

- [ ] 记录并固定 librmcs SDK 版本（3.3.0-alpha.7）与 SHA256；验证离线可构建
- [ ] 逐文件核对 v3 `hardware/device/` 与 v2 的协议换算一致性（D1–D17）
- [ ] 决策蜂鸣器：新写 `device/buzzer.hpp` + SDK 下行支持，或正式确认取消
- [ ] 决策 supercap 协议：`j_supercap.hpp` vs `supercap.hpp`，并同步各机型 config/组件
- [ ] 全机型 `usb_pid_*` → `board_serial_*` 参数替换，并获取真实板卡 serial
- [ ] 清理 `plugins.xml` 中的重复注册（`JOmniWheelController`）
- [ ] 统一 `/…/velocity_filtered` 消费点（PID 配置、broadcaster 名单）
- [ ] 若需 CAN3/UART4-6/dbus 以外通道，评估改线或扩展 SDK spec（未核实具体使用方）
- [ ] 回归：`./.script/build-rmcs-cross --target-arch=arm64 --packages-skip rmcs_auto_aim_v2 --executor sequential`

## 6. 验收与验证

- **编译**：交叉编译全量通过；`nm -C install-cross-arm64/lib/librmcs_core.so | grep registerPlugin` 能查到各 `hardware::*` 与控制器插件符号（已核实 `DualInfantry` 存在）。
- **静态核对**：`diff -u` 三份「完全一致」文件（benewake/gy614/hipnuc）应保持零差异；新驱动与 v2 librmcs 的换算常数逐项对齐。
- **链接**：`liblibrmcs-sdk.a` 被正确链接，运行时不依赖 `libusb-1.0.so`（应为静态）。
- **冒烟（需硬件）**：两块板卡按 serial 成功枚举；CAN 收发电机在线、DR16 遥控 `valid()`；IMU 出 `snapshot()`。
- **回归清单**：各机型 config 中引用的每个 `/…` 接口都能在硬件模型里找到对应的 register 目标（本主题与 06/07 交叉验证）。

## 7. 备注与未决问题

1. **`dji_motor_with_encoder.hpp` 位置**：实际在 `librmcs/librmcs/device/` 下，非 `hardware/device/`；v2 未被使用，v3 已删除。
2. **C 板 PID/libusb 细节**：v3 SDK 的 `host/src/transport/usb/*` 已编译进静态库，但 SDK 源码在 `_deps` 内；本主题未逐行核对 USB 协议帧（标注「未核实」）。
3. **蜂鸣器下位机支持**：v3 SDK C 板 spec 中未见 buzzer 字段，是否能通过其他字段（如 UART 透传 + 下位机自身蜂鸣）实现「未核实」。
4. **`Bmi088`（非 EKF）**：v3 保留但未被使用；是否保留待定（可能是给无 EKF 需求机型的备用）。
5. **`supercap.hpp` vs `j_supercap.hpp` 长期走向**：两套协议并存，需确立哪一套为 jRMCS 标准。
6. **SDK `AdvancedOptions.dangerously_skip_version_checks`**：v3 当前未使用；若下位机固件版本校验失败，需评估该开关（未核实下位机固件版本要求）。
7. **v2 失效插件注册**：`cboard::*`、`hardware::referee::Status` 等无实现却出现在 v2 `plugins.xml`；迁移时不要误当作「v2 有 v3 无」的功能。
