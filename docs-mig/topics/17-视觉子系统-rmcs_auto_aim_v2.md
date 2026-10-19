# 视觉子系统 rmcs_auto_aim_v2（自瞄）

- 状态：待办
- 优先级：P0
- 影响机型：dual-infantry（P0）、dual-sentry、mini-infantry、mec-hero、auto_aim_test（离线自瞄测试）
- 依赖主题：`04-消息与接口-rmcs_msgs与interface`（`CameraFrame` / `RobotId` / `/referee/*`）、`03-组件运行时-rmcs_executor`（in-process 插件、EventInputInterface）、`09-控制器-云台`（`/auto_aim/*` 消费方）、`18-bringup-配置与launch`（capturer 组件与 per-robot 配置装载）
- 验收方式：
  1. 交叉编译 `./.script/build-rmcs-cross --target-arch arm64`（若保留 `--packages-skip rmcs_auto_aim_v2`，需另做一次含自瞄的构建以验证）；
  2. 目标机 `set-robot dual-infantry && launch-rmcs` 后，`rmcs_executor` 内 `auto_aim_component` 正常加载，`/auto_aim/should_control|control_direction|should_shoot` 有数据、云台可被接管；
  3. 离线验证：`robot:=auto_aim_test` + `AutoAimVideoPlayerComponent` 回放视频，检测/跟踪/解算链路可跑通；
  4. `clang-format --dry-run -Werror` + `clang-format`/`clang-tidy` 通过。

---

## 1. v2（reATRM）现状

### 1.1 子模块版本

```
# git -C /workspaces/reATRM submodule status
 2c3122f8cb7200a8babe6db31d1b2e9b09644bf7 src/core/utils/system_ctrl/rmcs_auto_aim_v2 (remotes/origin/dev/25rmul)
```

- 路径：`/workspaces/reATRM/src/core/utils/system_ctrl/rmcs_auto_aim_v2/`
- 远端：`https://github.com/JinGangRobo/rmcs_auto_aim_v2`（`git remote -v` 仅 `origin`）
- 检出：`HEAD detached at 2c3122f`（= `origin/dev/25rmul` tip），提交时间 `2026-03-26`，作者 `MIDE酱`，标题 `优化自瞄, 提升准确率和健壮性 - 使用dual-infantry测试`
- 工作区**干净**（`git status --short` 无输出）
- 说明：`2c3122f` **不是** `origin/main-jrmcs` 的祖先；`origin/main-jrmcs` 是另一条线（见 §3.8）

### 1.2 目录结构（关键）

```
rmcs_auto_aim_v2/
├── CMakeLists.txt          # OpenCV 4.5 + OpenVINO + hikcamera 包；独立可执行 runtime
├── config/{dual-infantry,dual-sentry,mec-hero,mini-infantry}.yaml   # 每机型一份
├── launch/launch.py        # 起 rmcs_auto_aim_v2_runtime
├── plugins.xml             # 只注册 rmcs::AutoAimComponent（Control 角色）
├── models/{shenzhen-0526.onnx,shenzhen-0708.onnx,tongji-yolov5.xml,.bin}
├── src/runtime.cpp         # 独立进程 main()（自瞄主循环）
├── src/component.cpp       # rmcs_executor 插件：与 runtime 通过共享内存通信的“Control 角色”
├── src/kernel/{capturer,identifier,tracker,pose_estimator,fire_control,visualization,feishu,common,transform_tree}.{hpp,cpp}
├── src/module/{capturer,identifier,tracker,predictor,fire_control,debug,pipeline}/
└── src/utility/{image,math,rclcpp,shared,robot,thread,...}
```

### 1.3 架构：**两个进程 + 共享内存**

- **自瞄主进程**：`src/runtime.cpp` → 可执行 `rmcs_auto_aim_v2_runtime`（`CMakeLists.txt` 的 `add_executable(${PROJECT_NAME}_runtime src/runtime.cpp)`），由 `rmcs_auto_aim_v2/launch/launch.py` 拉起；`main()` 里按顺序构造 `Capturer/Identifier/Tracker/PoseEstimator/FireControl/Visualization` 并读配置（`runtime.cpp:53-109`）。
- **机器人进程内的桥接插件**：`src/component.cpp` 的 `rmcs::AutoAimComponent`（注册在 `plugins.xml`），`update()` 只做三件事：把 `rmcs_core` 的 TF/裁判信息打包成 `ControlState` 写入共享内存；从共享内存读 `AutoAimState` 并转成 `/gimbal/auto_aim/*` 接口（`component.cpp:71-107`）。
- **通信机制**：`src/kernel/feishu.hpp`，共享内存名 `/shm_autoaim_state`、`/shm_control_state`（`feishu.hpp:9-20`）；数据结构定义在 `src/utility/shared/context.hpp`（`AutoAimState`：`shoot_permitted/yaw/pitch/target/target_position[3]/selected_scan_direction`；`ControlState`：`shoot_mode/bullet_speed/yaw/pitch/invincible_devices/self_color/odom_to_{camera,base,muzzle}_transform`）。

### 1.4 `AutoAimComponent` 注册的接口（v2，`src/component.cpp:22-53`）

| 方向 | 名称 | 类型 | 备注 |
|---|---|---|---|
| input | `/tf` | `rmcs_description::Tf` | 必需；未 ready 则警告并退出本周期 |
| input | `/referee/id` | `rmcs_msgs::RobotId` | 决定敌我颜色 |
| input | `/referee/shooter/initial_speed` | `double` | **实时弹速** |
| output | `/gimbal/auto_aim/target_position` | `Eigen::Vector3d` | 目标位置（base 系） |
| output | `/gimbal/auto_aim/available` | `bool` | 云台可接管 |
| output | `/gimbal/auto_aim/control_direction` | `Eigen::Vector3d` | 单位方向向量 |
| output | `/gimbal/auto_aim/shoot_permit` | `bool` | 允许发弹（`aim_error < 0.002`，`component.cpp:125,186`） |
| output | `/debug/aim/{x,y,z,delay}` | `double` | 调试 |

- 消费方（v2 `rmcs_core`）：`controller/gimbal/simple_gimbal_controller.cpp:48-50`（`/gimbal/auto_aim/{available,control_direction,scan_direction}`）、`controller/shooting/bullet_feeder_controller_17mm.cpp:56`（`/gimbal/auto_aim/shoot_permit`）、`controller/diagnosis.cpp:71`、各机型 UI（`referee/app/ui/{dual_infantry,dual_sentry,mini_infantry,hero}.cpp` 读 `/gimbal/auto_aim/available`）。
- `/gimbal/auto_aim/scan_direction` 在 v2 `rmcs_core` 有声明（`simple_gimbal_controller.cpp:50,97-106`）但**本子模块没有任何 output 注册它**，`AutoAimState.selected_scan_direction` 也没有被 `component.cpp` 转成接口 → 属半接线/由 autopilot 侧消费（**未核实**）。

### 1.5 配置（per-robot + 环境变量）

- 由 `src/utility/rclcpp/configuration.hpp:10-22` 决定：**环境变量 `RMCS_ROBOT_TYPE` 存在时读 `<RMCS_ROBOT_TYPE>.yaml`**，否则读 `config.yaml`；位置为 `ament_index_cpp::get_package_share_directory("rmcs_auto_aim_v2")`（`parameters.cpp:10`）。
- 即：`set-robot dual-infantry` → `dual-infantry.yaml` 生效（与 bringup 同一开关）。
- 四份 config 的顶层键完全一致：`use_visualization / use_painted_image / is_local_runtime / capturer / identifier / tracker / pose_estimator / fire_control / visualization`。
- `capturer` 段（`dual-infantry.yaml:5-33`）：`source: hikcamera|local_video`、`hikcamera.{timeout_ms,exposure_us,framerate,gain,invert_image,software_sync,trigger_mode,fixed_framerate}`、`local_video.{location,frame_rate,loop_play,allow_skipping}`、`show_loss_framerate*`、`reconnect_wait_interval`。
- `identifier`（`:35-52`）：`binarization_threshold`、`avaliable_models[]`、`model_location`、`infer_device`（`AUTO`）、`use_roi_segment/roi_rows/roi_cols/input_rows/input_cols/min_confidence/score_threshold/nms_threshold`。
- `tracker`（`:54-56`）：仅 `enemy_color: blue|red`（静态敌我色）。
- `pose_estimator`（`:58-92`）：`camera_matrix[9]`（含 `fx/cx/fy/cy` 非对角项！如 dual-infantry `-1.205591323106101e-01`）、`distort_coeff[5]`、`transforms[]`（`imu_link→pitch_link→{muzzle_link,camera_link}`，全部 t=0/q=identity 占位）。
- `fire_control`（`:94-108`）：`initial_bullet_speed`、`shoot_delay`、`shoot_offset_{x,y,z}`、`k`、`bias_scale`、`coming_angle/leaving_angle/outpost_coming_angle/outpost_leaving_angle/angular_velocity_threshold`。
- `visualization`（`:110-114`）：`framerate / monitor_host / monitor_port / stream_type`（`RTP_JEPG`）。

### 1.6 构建与依赖

- `CMakeLists.txt`：`find_package(OpenCV 4.5)`、`find_package(OpenVINO REQUIRED)`、`find_package(hikcamera REQUIRED)`（独立 ROS 包 `/workspaces/reATRM/src/core/drivers/hikcamera`）、`rmcs_description`、`fast_tf`；编译产物 `rmcs_auto_aim_v2_{utility,kernel,module,component}.so` + `rmcs_auto_aim_v2_runtime`。
- `package.xml`：依赖 `rmcs_executor / rmcs_description / rmcs_msgs / hikcamera`；**无** `rmcs_utility`、**无** `fast_tf` 显式声明（`fast_tf` 经 `rmcs_description` 传入）。

---

## 2. v3（jRMCS）现状

### 2.1 子模块版本

```
# git -C /workspaces/RMCS submodule status
 8df0266e0560759b4ba2acc3b1b48832a0ccc530 rmcs_ws/src/rmcs_auto_aim_v2 (heads/dev/rk3588npu)
# git -C /workspaces/RMCS ls-tree HEAD rmcs_ws/src/rmcs_auto_aim_v2
160000 commit 8df0266e0560759b4ba2acc3b1b48832a0ccc530
```

- 路径：`/workspaces/RMCS/rmcs_ws/src/rmcs_auto_aim_v2/`（`.gitmodules` url = `https://github.com/JinGangRobo/rmcs_auto_aim_v2.git`）
- 检出：本地分支 `dev/rk3588npu` = `origin/dev/rk3588npu` tip，提交时间 `2026-10-04`，作者 `MIDE酱`，标题 `feat(rknn): pipeline inference across two NPU cores`
- **工作区有未提交修改**（`git status --short`）：
  - `M CMakeLists.txt`：`set(RMCS_AUTO_AIM_INFER_BACKEND "openvino" ...)` → `"rknn"`（即改为 RK3588 NPU 推理）
  - `M config/config.yaml`：把上游 `camera_matrix`（`1.722231837421459e+03 / 7.013056440882832e+02 / ...`）与单行 `distort_coeff: [-0.064232403853946, -0.087667493884102, 0, 0, 0.792381808294582]` 替换为**与 v2 `dual-sentry.yaml` 完全一致**的内参与畸变（`1.758678222166633e+03 / 7.360359340831083e+02 / ...`、`[-5.986950903811911e-02, 1.011540116487305e-01, 0, 0, 0]`）
  - 这两处属于用户在建的实机适配工作，**本次不动**，但必须记录（迁移方案需决定是提交、还是改回上游默认）。

### 2.2 目录结构（关键）

```
rmcs_auto_aim_v2/
├── CMakeLists.txt          # 编译期推理后端切换（cmake/InferBackend.cmake + cmake/backend/*.cmake）
├── cmake/{InferBackend.cmake,infer_backend_config.hpp.in,backend/{openvino,rknn,example}.cmake}
├── config/{config.yaml,executor.yaml}    # config.yaml = 算法配置；executor.yaml = 独立自瞄参考用例
├── plugins.xml             # AutoAimComponent + Capturer + Recorder + Player + VideoPlayer
├── models/{shenzhen-0526.onnx,.rknn,_unq.rknn, shenzhen-0708.*, tongji-yolov5.xml/.bin}
├── src/component.cpp       # rmcs_executor 插件：自瞄全链路（in-process）
├── src/io/{capturer,recorder}/      # 采集/录制/回放组件
├── src/kernel/{auto_aim,detector,pose_estimator,tracker,fire_control,visualization,feishu,transform_tree,common}
├── src/module/{detector,perception,tracker,fire_control,debug}
└── src/utility/{...}
（**无 launch/、无 src/runtime.cpp**）
```

### 2.3 架构：**单进程 in-process 插件（+ 独立采集组件）**

- 没有独立 runtime 可执行；自瞄作为 `rmcs_executor` 插件随 bringup 的单一 `rmcs_executor` 节点加载（`AGENTS.md` 第 4 节）。
- `src/component.cpp` 的 `rmcs::AutoAimComponent` 内含 `AutoAim auto_aim`、`RclcppNode rclcpp`、`std::jthread camera_frame_worker`（`component.cpp:31-84`），在 worker 线程里 `auto_aim.process(Image{frame})`。
- **图像来自别的组件**：`src/io/capturer/capturer.cpp` 的 `AutoAimCapturerComponent` 直接持有海康相机（内部 `hikcamera.hpp`，依赖 `MVSUSB3Core::MVSUSB3Core`，见 `CMakeLists.txt`），通过 `register_output(frame_topic, frame_output_)` 发布 `/gimbal/auto_aim/camera_frame`（`capturer.cpp:66-86`），并可选订阅 `/gimbal/auto_aim/exposure_signal`、`/gimbal/auto_aim/imu_snapshot` 做硬件同步。
- `rmcs_msgs::CameraFrame`（`rmcs_ws/src/rmcs_msgs/include/rmcs_msgs/camera_frame.hpp`）随帧携带 `imu_snapshot`（`Eigen::Quaterniond`）、`gyro_body`、`exposure_timestamp`、`image_reception_timestamp`、`sync_publish_timestamp`。
- 推理后端：编译期选择 `none|openvino|rknn`（`cmake/InferBackend.cmake`，当前工作区设为 `rknn`，且模型目录新增 `*.rknn`/`*_unq.rknn`）。

### 2.4 `AutoAimComponent` 注册的接口（v3，`src/component.cpp:120-141`）

| 方向 | 名称 | 类型 | 备注 |
|---|---|---|---|
| input | `/gimbal/auto_aim/camera_frame` | `EventInputInterface<std::shared_ptr<const rmcs_msgs::CameraFrame>>` | 由 `AutoAimCapturerComponent` 发布 |
| input | `/referee/id` | `rmcs_msgs::RobotId` | 未 ready 时可用 `dangerous_fallback` 强绑 |
| input | `/remote/switch/right`、`/remote/switch/left` | `rmcs_msgs::Switch` | 右拨杆 UP=跟踪意图；左拨杆 DOWN=开火意图 |
| input | `/remote/mouse`、`/remote/keyboard` | `Mouse`/`Keyboard` | 右键/左键；F 键切符 |
| input | `/rmcs_navigation/request/track_rune`、`/rmcs_navigation/track_building_only` | `bool` | 导航请求打符 / 只打基地与前哨站 |
| output | `/auto_aim/should_control` | `bool` | 允许接管云台 |
| output | `/auto_aim/control_direction` | `Eigen::Vector3d` | 首发默认 `kNaN` |
| output | `/auto_aim/robot_center` | `Eigen::Vector3d` | 目标车中心 |
| output | `/auto_aim/should_shoot` | `bool` | 允许发弹 |
| output | `/auto_aim/single_shoot` | `bool` | 能量机关模式标志 |
| output | `/auto_aim/ff_v`、`/auto_aim/ff_a` | `Eigen::Vector3d` | 前馈速度/加速度 |

- 消费方（v3 `rmcs_core`）：`controller/gimbal/simple_gimbal_controller.cpp:35-36`、`controller/gimbal/hero_gimbal_controller.cpp:36-37`、`controller/gimbal/eccentric_dual_yaw.cpp:188-189`、`controller/gimbal/deformable_infantry_gimbal_controller.cpp:140-142`（读 `/auto_aim/{should_control,control_direction,robot_center}`）；`controller/shooting/bullet_feeder_controller_17mm.cpp:54-55`（`/auto_aim/{should_shoot,single_shoot}`）；`controller/shooting/putter_controller.cpp:78`；UI `referee/app/ui/auto_aim.cpp:35-37`（`/auto_aim/{robot_center,should_shoot,single_shoot}`，即插件类 `rmcs_core::referee::app::ui::AutoAimUi`）。
- `rmcs_auto_aim_v2/plugins.xml` 注册 5 个类：`rmcs::AutoAimComponent`、`AutoAimCapturerComponent`、`AutoAimRecorderComponent`、`AutoAimPlayerComponent`、`AutoAimVideoPlayerComponent`。
- **弹速是配置常量**：`kernel/fire_control.cpp:316,369` 用 `config.bullet_speed`；组件**没有**订阅 `/referee/shooter/initial_speed`（全文件 grep 无 `initial_speed`）。

### 2.5 配置（单份 `config.yaml` + 参数覆盖）

- `src/utility/rclcpp/configuration.hpp:12-50`：优先 `AUTOAIM_CONFIG` 环境变量 → 再依次找 `custom.yaml|custom.yml|config.override.yaml|config.override.yml|config.yaml|config.yml`（share 目录下），否则抛异常。
- 顶层键：`use_visualization / camera_matrix / distort_coeff / virtual_rune / detector / tracker / pose_estimator / visualization`（`config/config.yaml`）。
  - **`camera_matrix` / `distort_coeff` 提升为顶层**（v2 在 `pose_estimator` 内）。
  - `virtual_rune`（新增）：`enable/large/x/y/z/face_yaw`。
  - `detector`（v2 的 `identifier` 改名）：新增 `green_light_filter{green_threshold,min_area,min_circularity,max_aspect_ratio}`；`infer_device: "CPU"`（v2 为 `AUTO`）；`score_threshold: 0.7`、`nms_threshold: 0.3`（v2 为 `0.8/0.45`）；**删** `binarization_threshold`、`avaliable_models[]`。
  - `tracker`：`fallback_color`/`timeout_seconds`/`image_margin` + `robot.*`（EKF 噪声）+ `outpost.*`（EKF 噪声、`plate_switch_yaw_min`）（v2 只有 `enemy_color`）。
  - `pose_estimator`：`distance_optimizer / fixed_outpost_pitch / fixed_normal_pitch / outpost_armor_thickness`（`transforms[]` 取消，改为 `component.cpp` 的 `camera_translation` 参数 + `kernel/transform_tree.hpp` 内置 `pitch_link/muzzle_link/camera_link`）。
  - `visualization`：新增 `publishable / drawable / enable_stream`；`stream_type: "RTP_H264"`（v2 为 `RTP_JEPG`）。
- **组件参数**由 ROS 参数提供（`component.cpp:142-213`）：`manual_shoot`（**必填**）、`enable_rune`、`default_attack_rune`、`dangerous_fallback`、`track_ids`、`camera_translation`、`fire_control.*`（`kernel/fire_control.hpp:16-53`：`bullet_speed/shoot_delay/offset_yaw/offset_pitch/attack_window/degraded_angle_speed/window_redundancy/window_hysteresis/yaw_tolerance/pitch_tolerance/attack_preaim/rune_idle_duration/rune_shoot_duration`）。
- `config/executor.yaml`：独立自瞄用例的参考配置，示例 `components: [- rmcs::AutoAimComponent -> auto_aim_component]`，并给 `capturer_main / auto_aim_recorder / auto_aim_player / auto_aim_video_player / auto_aim_component` 各写了参数块。

### 2.6 构建与依赖

- `CMakeLists.txt`：`find_package(MVSUSB3Core REQUIRED CONFIG PATHS "${HIK_MVS_SDK_ROOT}" NO_DEFAULT_PATH)`（海康 MVS usb3-only SDK，非 ROS 包）；`rmcs_msgs / rmcs_executor / rmcs_description / rmcs_utility / fast_tf`；分三个库 `${PROJECT_NAME}_utility/_kernel/_module` + `${PROJECT_NAME}_component`。
- `package.xml`：新增 `fast_tf`、`std_msgs`、`rmcs_utility`、`ament_cmake_gtest`；仍声明 `hikcamera`（**v3 工作区已无 `hikcamera` 包**，见 `rmcs_ws/src` 的 8 个包 —— 该依赖为遗留声明，**未核实**是否会导致解析告警）。

---

## 3. 差异清单

### 3.0 子模块版本差异（权威数据）

```
# 在 RMCS 的 rmcs_auto_aim_v2 克隆内（两个 commit 都在）
$ git merge-base 2c3122f 8df0266
ffebdb74522dac76ae383330a120a68df1f4417c      # "Fix/18 coordinate transform and fixmes (#19)"
$ git rev-list --left-right --count 2c3122f...8df0266
23      174                                    # v2 独有 23 个提交 / v3 独有 174 个提交
$ git diff --stat 2c3122f 8df0266 | tail -1
246 files changed, 20837 insertions(+), 5355 deletions(-)
```

分支归属（`git branch -a` + tip 查询）：

| 引用 | tip | 与本任务关系 |
|---|---|---|
| `origin/dev/25rmul` | `2c3122f` | **v2 实际使用版本**（优化自瞄/dual-infantry 调参） |
| `origin/dev/rk3588npu` | `8df0266` | **v3 实际使用版本**（NPU 推理） |
| `origin/main-jrmcs` | `4f43409` | v3 的祖先（`git merge-base --is-ancestor origin/main-jrmcs 8df0266` → yes，落后 6 个提交） |
| `origin/main` | `4f43409` | 同上 |
| `origin/develop` | — | 未核实 |

**结论（是否需要同步子模块版本）**：

- v3 分支 `dev/rk3588npu` **已经包含** `main-jrmcs`/`main`（`4f43409`），但**不包含** v2 的 23 个 `dev/25rmul` 提交；两者自 `ffebdb7` 分叉，属**实质性分叉**，不能简单“同步”：
  - `dev/rk3588npu` 新：仓库级重构（`kernel/detector`+`module/*`、`io/capturer`、`auto_aim` kernel、插件化组件、RKNN/OpenVINO 后端切换、能量机关/虚拟符、录制回放、可视化重写、`AGENTS.md`、`tool/cxx` 工具集）。
  - `dev/25rmul` 独有：面向 reATRM 实车的 23 个提交（`add dynamic enemy color check`、`add target position transmit`、`add a simple shoot permit deciding`、`change bullet_speed interface type`、`update {sentry,mini-infantry,mec-hero,dual-infantry} params`、`优化自瞄…使用dual-infantry测试` 等）。
- **建议**：**不整体切换子模块版本**，而是按 §4 把 v2 `dev/25rmul` 的**功能语义**（而非代码）移植到 v3 `dev/rk3588npu` 之上；子模块内部新增提交须推回 `JinGangRobo/rmcs_auto_aim_v2`（当前 v3 工作区已有 2 处未提交改动，见 §2.1）。

### 3.1 架构差异（最大差异）

| 差异点 | v2 做法 | v3 做法 | 影响 | 风险 |
|---|---|---|---|---|
| 进程模型 | 自瞄=独立进程 `rmcs_auto_aim_v2_runtime` + 插件桥接 | 自瞄=in-process 插件，随 `rmcs_executor` 加载 | 启动方式、日志、调试端口全变 | 高：v2 的实车调参经验需要在新进程模型下重测 |
| 进程间通信 | 共享内存 `feishu`（`/shm_autoaim_state`、`/shm_control_state`，`kernel/feishu.hpp`） | 无；组件内直接调用 + ROS 接口 | v2 的 `ControlState/AutoAimState` 被 `rmcs_msgs` 接口取代 | 中 |
| 数据入参 | `/tf`（`rmcs_description::Tf`）+ 实时弹速 | `CameraFrame{imu_snapshot,gyro_body,exposure_timestamp}` + 配置常量弹速 | v3 依赖采集端提供同步后的姿态 | 高：姿态时间戳对齐是自瞄精度关键 |
| 相机驱动 | 独立 ROS 包 `hikcamera`（`/reATRM/src/core/drivers/hikcamera`） | 内部 `io/capturer/hikcamera.hpp` + `MVSUSB3Core` SDK | 依赖链与部署方式不同（SDK 需进 sysroot/镜像） | 中 |
| 推理后端 | OpenVINO（`AUTO` 设备） | 编译期 `none|openvino|rknn`，v3 工作区当前＝`rknn` | 模型格式不同（`.onnx/.xml` vs `.rknn`），精度需重标定 | 中 |

### 3.2 接口差异（与主链路 coupling point，**迁移重点**）

| v2 接口 | v3 接口 | 说明 |
|---|---|---|
| `/gimbal/auto_aim/available` | `/auto_aim/should_control` | 云台可接管 |
| `/gimbal/auto_aim/control_direction` | `/auto_aim/control_direction` | 语义一致，命名空间变了 |
| `/gimbal/auto_aim/shoot_permit` | `/auto_aim/should_shoot` | 发弹许可 |
| `/gimbal/auto_aim/target_position` | `/auto_aim/robot_center` | 目标位置 |
| `/gimbal/auto_aim/scan_direction`（无人写） | **无** | v3 无扫描方向接口 |
| `/referee/shooter/initial_speed`（输入） | **无**（改用 `fire_control.bullet_speed`） | 弹速实时性/热插拔能力回退 |
| — | `/auto_aim/single_shoot`、`/auto_aim/ff_a`、`/auto_aim/ff_v` | v3 新增（能量机关、前馈） |
| `/tf` | `CameraFrame.imu_snapshot` | 外参来源改变 |
| `/debug/aim/{x,y,z,delay}` | —（由 `visualization` / `referee/app/ui/auto_aim.cpp` 替代） | 调试 topic 变了 |

- 同一插件类名 `rmcs::AutoAimComponent`、同一库名 `rmcs_auto_aim_v2_component`，但**语义完全不同**（v2=shm 桥，v3=全链路），排障时极易混淆。
- v3 的 `rmcs_core` 侧消费点已全部改成 `/auto_aim/*`（`simple_gimbal_controller.cpp:35-36` 等），故**不能**直接把 v2 的 `/gimbal/auto_aim/*` 接口移植回 v3 而不改核心。

### 3.3 与 v3 主链路对接的**缺口**（必须核实，可能阻塞）

| 缺口 | 证据 | 影响 | 风险 |
|---|---|---|---|
| 没有任何 bringup 配置加载 `AutoAimCapturerComponent` | 全工作区 grep `AutoAimCapturer` 仅命中 `rmcs_auto_aim_v2/`（`plugins.xml`、`config/executor.yaml`）；`rmcs_bringup/config/*.yaml` 只列 `rmcs::AutoAimComponent` | `/gimbal/auto_aim/camera_frame` **无发布者** → 自瞄拿不到图像 | 高 |
| `rmcs_core` 不产生相机触发/IMU 快照 | grep `exposure_signal`/`camera_frame` 在 `rmcs_core` 无命中（`dual_infantry.cpp:163,324` 只是本地 IMU EKF，未输出快照 topic） | `use_hardware_sync: true` 时 capturer 会因缺输入退化为自由采集 | 高（**未核实** `rmcs_msgs` 是否有对应输出组件） |
| 单份 `config.yaml` 取代 4 份 per-robot 配置 | v3 `configuration.hpp` 无 `RMCS_ROBOT_TYPE` 分支 | dual-infantry 的相机内参/畸变（曾按机型区分）退化到一份全局配置；当前工作区改的正是这一份（对齐 dual-sentry） | 高 |
| 弹速来源改为配置常量 | `fire_control.hpp:17` + `component.cpp` 无 `initial_speed` 输入 | 机器人弹速变化（裁判数据）不再被自瞄使用 | 中 |
| `dual-infantry.yaml` 缺 `auto_aim_component` | 见主题 `18` §3 | P0 机型默认不带自瞄 | 高（跨主题） |

### 3.4 配置文件差异

| 项 | v2 | v3 | 影响 |
|---|---|---|---|
| 文件数/命名 | `{dual-infantry,dual-sentry,mec-hero,mini-infantry}.yaml` | `config.yaml` + `executor.yaml` | 机型差异只能靠 `AUTOAIM_CONFIG`/override 恢复 |
| 选择机制 | `RMCS_ROBOT_TYPE` → `<robot>.yaml` | `AUTOAIM_CONFIG` → `custom.yaml` → `config.override.yaml` → `config.yaml` | 与 `set-robot` 不再联动 |
| 关键段名 | `identifier`、`enemy_color` | `detector`、`tracker.fallback_color` | 需人工改写 |
| 内参位置 | `pose_estimator.camera_matrix/distort_coeff` | 顶层 `camera_matrix/distort_coeff` | 同上 |
| 外参 | `pose_estimator.transforms[]` | `auto_aim_component.camera_translation[3]` + `camera_link/muzzle_link` 内置 | 外参表达方式变化 |
| 标定值 | 4 机型 4 组内参 | 1 组（工作区未提交，值 = v2 dual-sentry 组） | 实车需按机型重新标定/配置 |
| `fire_control` | `initial_bullet_speed/shoot_offset_*/k/bias_scale/coming_angle/...` | `bullet_speed/offset_yaw/offset_pitch/attack_window/degraded_angle_speed/window_redundancy/...` | 参数体系重写（弹道模型不同） |
| 阈值 | `score_threshold 0.8`、`nms 0.45`、`infer_device AUTO` | `0.7`、`0.3`、`CPU` | 检出率/误检需重调 |
| `visualization` | `RTP_JEPG` | `RTP_H264` + `publishable/drawable/enable_stream` | 直播链路不同 |

### 3.5 文件级差异（`git diff --name-status 2c3122f 8df0266`，v2→v3）

**新增（A，节选关键项）**
- 仓库级：`AGENTS.md`、`cmake/InferBackend.cmake`、`cmake/backend/{example,openvino,rknn}.cmake`、`cmake/infer_backend_config.hpp.in`、`config/config.yaml`、`config/executor.yaml`、`doc/rknn.md`
- 采集/回放：`src/io/capturer/{capturer.cpp,hikcamera.hpp,imu_snapshot_buffer.hpp,linear_sync_model.hpp}`、`src/io/recorder/{recorder.cpp,player.cpp,video_player.cpp}`
- kernel：`src/kernel/{auto_aim.cpp,auto_aim.hpp,detector.cpp,detector.hpp}`
- module：`src/module/detector/{adjacency_lightbar,green_light,lightbar,rune}.{hpp,cpp}`、`src/module/perception/rotation_estimator.{hpp,cpp}`、`src/module/tracker/model/{outpost,robot,rune,rune_energy_fitter,virtual_rune}.{hpp,cpp}`、`src/module/tracker/trackable.hpp`、`src/module/fire_control/shoot_evaluator.{hpp,cpp}`
- utility：`src/utility/math/{camera,corners_optimizor,outpost,reprojection,robot}.{hpp,cpp}`、`src/utility/math/solve_pnp/{outpost_distance_optimizer,yaw_optimizer}.{hpp,cpp}`、`src/utility/math/{hungarian,mahalanobis}.hpp`、`src/utility/model/infer_backend.*`、`src/utility/model/backend/{openvino_backend,rknn_backend}.{hpp,cpp}`、`src/utility/image/{drawable,process,recorder,text}.{hpp,cpp}`、`src/utility/rclcpp/visual/{arrow,lightbar,point,scalar}.{hpp,cpp}`、`src/utility/{framerate,logging_util,repeat,service,string}.hpp`、`src/utility/csv/csv_reader.hpp`、`src/utility/robot/{aimpoint,rune}.hpp`
- 模型：`models/shenzhen-0526.rknn`、`shenzhen-0526_unq.rknn`、`shenzhen-0708.rknn`、`shenzhen-0708_unq.rknn`
- 工具：`tool/cxx/*`（`CMakeLists.txt`、`hikcamera.cpp`、`hikcamera_test.cpp`、`rotation_estimator.cpp`、`see_outpost.cpp`、`test_outpost_ekf.cpp`、`test_robot_ekf.cpp`、`util/*`、`visualization.cpp`）、`tool/install-server.sh`、`tool/res/*`

**删除（D）**
- `config/{dual-infantry,dual-sentry,mec-hero,mini-infantry}.yaml`、`launch/launch.py`、`src/runtime.cpp`
- `src/kernel/{capturer,identifier}.{hpp,cpp}`、`src/utility/shared/context.hpp`（被 `rmcs_msgs` 取代）、`src/utility/image/{armor.hpp,armor.cpp,image.cpp,image.details.hpp}`、`src/utility/math/point.hpp`
- `src/module/capturer/*`（`common/hikcamera/image/local_video/runtime/video`）、`src/module/identifier/{armor,classifier,lightbar,preprocess}.{hpp,cpp}`、`src/module/predictor/{ekf_parameter.hpp,robot_state.*,snapshot.*}`、`src/module/tracker/{armor_filter.*,decider.*,state.hpp}`、`src/module/fire_control/aim_point_chooser.*`、`src/module/debug/{recorder.*,framerate.hpp}`、`src/module/debug/visualization/armor_visualizer.*`、`src/module/pipeline/common.hpp`
- `tool/{CMakeLists.txt,hikcamera.cpp,streaming.cpp}`、`test/{action_throttler.cpp,feishu_test.cpp,pipeline.cpp}`

**重命名/移动（R）**
- `src/module/identifier/armor_detection.{cpp,hpp}` → `src/module/detector/armor_detection.{cpp,hpp}`（R064/R068）
- `src/module/identifier/models/*` → `src/module/detector/models/*`（R055/R100/R059）
- `src/module/debug/framerate.hpp` → `src/utility/framerate.hpp`（R091）
- `tool/ov_model_status.cpp`→`tool/cxx/…`、`tool/util/*`→`tool/cxx/util/*`、`tool/visualization.cpp`→`tool/cxx/visualization.cpp`

**修改（M，关键项）**
- `CMakeLists.txt`（依赖与目标重构）、`package.xml`、`plugins.xml`（1 类 → 5 类）、`src/component.cpp`（全面重写）、`src/kernel/{fire_control,pose_estimator,tracker,visualization}.{hpp,cpp}`、`src/kernel/common.hpp`、`src/kernel/feishu.hpp`、`src/utility/rclcpp/{configuration,parameters,node,param_adapter}.{hpp,cpp}`、`src/utility/shared/interprocess.hpp`、`src/utility/tf/static_tf.hpp`、`src/utility/yaml/tf.hpp`、`.clang-format`、`.gitignore`

**规模**：246 files changed, +20837 / -5355。

### 3.6 需要**保留回迁**的 v2 语义（`dev/25rmul` 23 提交）

按 `git log --oneline 8df0266..2c3122f`（v2 独有）整理，可移植的“功能语义”包括：

| v2 提交 | 语义 | v3 现状 |
|---|---|---|
| `755a895 add dynamic enemy color check` | 敌我色动态判定（不只依赖静态 `enemy_color`） | v3 `tracker.fallback_color` + `RobotId` 驱动（**部分等价，未核实**） |
| `9daa9e0 change bullet_speed interface type` | 弹速走接口（实时） | v3 为配置常量（缺口） |
| `99ec8eb add a simple shoot permit deciding` | 简化的开火许可判定 | v3 `fire_control` 的 `shoot`/`attack_window` 体系（语义不同） |
| `f7f7a83 add target position transmit` | 目标位置上传 | v3 `/auto_aim/robot_center` 已有 |
| `2eb09b1 update a lite version of auto select scan direction` | 自动选择扫描方向 | v3 无（`scan_direction` 接口消失） |
| `a25be28 / 0eb48b8 / 5c91108 / 13bf274 update {mini-infantry,mec-hero,sentry} params`、`2c3122f 优化自瞄…使用dual-infantry测试` | 4 机型自瞄调参 + dual-infantry 实车标定 | v3 只有一份 `config.yaml`，需重建机型参数集 |

---

## 4. 迁移方案（以 v3 为骨架）

> 原则：**不改 v3 架构**（保持自瞄为 in-process 插件 + 独立 capturer 组件 + `/auto_aim/*` 接口），把 v2 的**可用语义与实车参数**“翻译”进 v3。

1. **恢复 per-robot 自瞄配置（P0，最小改动）**
   - 方案 A（推荐）：在 v3 的 `src/utility/rclcpp/configuration.hpp` 中，在 `AUTOAIM_CONFIG` 之后、`config.yaml` 之前，插回 `RMCS_ROBOT_TYPE` 分支（`location / (getenv("RMCS_ROBOT_TYPE") + ".yaml")`），并在 `config/` 下新增 `dual-infantry.yaml / dual-sentry.yaml / mini-infantry.yaml / mec-hero.yaml`（由 v3 的 `config.yaml` 复制后按机型改内参）。
   - 方案 B：不改代码，靠 `AUTOAIM_CONFIG=<abs path>`（`launch-rmcs` 前 export）。
   - 保留 `config.yaml` 作为默认兜底。
2. **把 capturer 接进 bringup（P0，阻塞项）**
   - 在 `rmcs_bringup/config/<robot>.yaml` 的 `components:` 加入 `- rmcs::AutoAimCapturerComponent -> auto_aim_capturer`，并把 `auto_aim_capturer: { ros__parameters: {camera_name, exposure_us, gain, framerate, invert_image, use_hardware_sync, delay_ms, rls_tau_sec, frame_topic, exposure_signal_topic, imu_snapshot_topic} }` 按机型填好（参数名取自 `src/io/capturer/capturer.cpp:31-56`）。
   - 明确 `use_hardware_sync: false` 的 fallback（当前 `rmcs_core` 无触发/快照输出，见 §3.3）。
3. **弹速实时化**
   - 在 `component.cpp` 增加 `InputInterface<float> bullet_speed`（`/referee/shooter/initial_speed`，v3 `referee/status.cpp:82` 已有输出）并在 `fire->update()` 前覆盖 `config.bullet_speed`；或在 `FireController::State` 增加 `bullet_speed` 字段。
4. **移植 v2 机型参数（P1）**
   - 逐机型把 v2 `config/<robot>.yaml` 的 `identifier/tracker/pose_estimator/fire_control/visualization` 翻译为 v3 `detector/tracker/pose_estimator/fire_control/visualization` 键（对照 §3.4 表）。
   - 注意 v3 `fire_control` 是**弹道迭代解算**（`module/fire_control/trajectory_solution.cpp`），v2 的 `k/bias_scale/coming_angle/...` 无直接对应项，不能机械照抄。
5. **接口补齐（P2）**
   - 扫描方向：若实车需要，在 v3 的 `rmcs_core` 侧（`SimpleGimbalController` 扫描逻辑）与自瞄之间新增 `/auto_aim/scan_direction`（或复用 `Context.addition`），并在 `component.cpp` 输出。
6. **清理与防混用**
   - 明确 `config/executor.yaml` 只是参考用例；把它改名为 `executor.example.yaml`（或加注释）以免被误当 bringup 配置。
   - 修正 `package.xml` 中已不存在的 `hikcamera` 依赖（改为 SDK/`rmcs_utility` 说明）。
   - `auto_aim_test.yaml` 中的 `require_stable_command`（`config/auto_aim_test.yaml:48`）在 v3 源码中**不存在对应键**（grep 无命中），删除或补齐实现。
7. **上游回推**：把 v3 工作区两处未提交改动（`CMakeLists.txt` 后端切 `rknn`、`config/config.yaml` 内参）确认后提交并推回 `JinGangRobo/rmcs_auto_aim_v2`，否则后续 `git submodule update` 会丢。

---

## 5. 执行步骤 checklist

- [ ] 记录并确认 v3 子模块两处**未提交**改动（`CMakeLists.txt` 的 `RMCS_AUTO_AIM_INFER_BACKEND=rknn`、`config/config.yaml` 内参/畸变）——提交或回退
- [ ] `rmcs_bringup/config/dual-infantry.yaml`（及双哨兵等）加入 `rmcs::AutoAimCapturerComponent` 并配齐 `auto_aim_capturer` 参数块
- [ ] 决定并实现 per-robot 自瞄配置装载（`RMCS_ROBOT_TYPE` 分支 或 `AUTOAIM_CONFIG`）
- [ ] 由 v3 `config.yaml` 派生 4 份机型自瞄配置，填入 v2 标定的内参/畸变（逐机型）
- [ ] 弹速改为 `/referee/shooter/initial_speed` 实时输入（保留配置兜底）
- [ ] 翻译 v2 → v3 的 `detector/tracker/pose_estimator/fire_control` 参数（逐机型，标注无法对应的键）
- [ ] 明确 `use_hardware_sync` 策略并验证（含无触发信号时的行为）
- [ ] `auto_aim_test.yaml`：删除 `require_stable_command`；用 `AutoAimVideoPlayerComponent` 跑通离线链路
- [ ] （可选）补 `/auto_aim/scan_direction`
- [ ] `clang-format` / `clang-tidy` 通过
- [ ] arm64 交叉编译通过（含自瞄包）
- [ ] 更新本文件状态与 `docs-mig/README.md` 进度

---

## 6. 验收与验证

1. **构建**：`./.script/build-rmcs-cross --target-arch=arm64`（**不加** `--packages-skip rmcs_auto_aim_v2`）通过；`nm -C install-cross-arm64/lib/librmcs_auto_aim_v2_component.so | grep registerPlugin` 能列出 5 个类。
2. **离线**：`ros2 launch rmcs_bringup rmcs.launch.py robot:=auto_aim_test` → `AutoAimVideoPlayerComponent` 播 `/workspaces/data/autoaim/静止看前哨站.avi`，观察 `visualization` 是否出流、`/auto_aim/*` 是否随帧刷新。
3. **配置装载**：`set-robot dual-infantry` 后启动，日志应出现 `configuration.hpp:24/44` 的“已作为配置文件加载 <path>”，且 path 指向 `dual-infantry.yaml`。
4. **实机链路**：dual-infantry 上 `rmcs_executor` 内 `auto_aim_component` + `auto_aim_capturer` 同时加载；右键取出云台接管（`/auto_aim/should_control=true`），`/auto_aim/control_direction` 非 NaN；`/auto_aim/should_shoot` 可触发 `BulletFeederController17mm`（`bullet_feeder_controller_17mm.cpp:54-55`）。
5. **回归**：`simple_gimbal_controller` / `hero_gimbal_controller` / `eccentric_dual_yaw` 的手动控制不受影响（`auto_aim_control_direction` 在未跟踪时应为 NaN/零）。

---

## 7. 备注与未决问题

1. **dual-infantry 是 P0**：v2 最后一次自瞄调参提交标题即“使用dual-infantry测试”，而 v3 当前 `dual-infantry.yaml` 里**没有** `auto_aim_component`（见主题 `18`）——两份文档需一起执行。
2. **`/gimbal/auto_aim/scan_direction`**：v2 `rmcs_core` 读取但 v2 子模块无输出；不确定是否由 `autopilot_client` 侧填写（**未核实**，`autopilot_client` 不在本次范围）。
3. **`selected_scan_direction` / `target` / `shoot_mode`** 等 shm 字段在 v3 语义下的落点（是否需要新接口）**未决**。
4. **v3 `package.xml` 仍声明 `hikcamera`**，但 `rmcs_ws/src` 无该包 —— 是否影响 `rosdep`/构建顺序**未核实**（CI 若用 `rosdep install` 可能报错）。
5. **v3 `models/` 通过 `git-lfs` 还是直存**、`.rknn` 模型是否需要随包分发**未核实**（`models/*.onnx` 与 `*.rknn` 均在仓库内）。
6. **v3 推理后端若为 `rknn`**：`detector` 配置里的 `infer_device: "CPU"` 对 RKNN 后端是否生效**未核实**（`armor_detection.cpp:170-183` 仅在非空时覆盖 `model.infer_device`）。
7. `git diff` 数据来自 `/workspaces/RMCS/rmcs_ws/src/rmcs_auto_aim_v2` 的本地克隆（同一远端，两个 commit 均可用），命令：`git diff --name-status 2c3122f 8df0266`。

---

## 迁移状态（阶段二，2026-10-19）

**已完成（rmcs_core 侧 + config 接入）。** 自瞄子模块本体未改动。
- `hardware/dual_infantry.cpp` 新增 `/gimbal/auto_aim/imu_snapshot`（必开）与可选 `/gimbal/auto_aim/exposure_signal`（`camera_sync_gpio_channel` 控制，默认 -1 关闭）。
- `dual-infantry.yaml` 接入 `rmcs::AutoAimCapturerComponent`（`use_hardware_sync: false`）。
- **未验证/待办**：`AutoAimCapturerComponent` 依赖 `hikcamera`，因 `--packages-skip rmcs_auto_aim_v2` 未编译；相机内参走 `AUTOAIM_CONFIG`。
