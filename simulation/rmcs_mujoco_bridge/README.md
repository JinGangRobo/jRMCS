# 七轴 MuJoCo 仿真

`MuJoCoBridge` 在仿真配置中替代硬件组件。它给 `ArmConfig` 提供七轴角度、角速度和已施加力矩，再由命令伙伴组件将 `ArmSolver` 的输出施加到 MuJoCo。`PositionTest` 提供固定目标，待反馈有效后使能，达到指定仿真时长时输出结果并退出。

模型从 `arm_description` 的 `JGarm.urdf` 读取。仿真保留 `Link8`、`Link9` 和两个滑动关节，从 MoveIt 的 `Gripper_open` 开位启动（`joint8=-0.032 m`、`joint9=+0.032 m`）。固定目标测试和容器内 Qt 仿真将两指固定在开位。仿真使用独立力矩上限并关闭接触，隔离网格重叠对位置跟踪的影响。默认每 1 ms 推进一步，七轴力矩分两组交替更新，各轴每 2 ms 更新一次。

## 在当前 devcontainer 中运行

当前容器的默认终端是 zsh。使用 `setup.zsh` 和 `local_setup.zsh`，从 `/workspaces/RMCS/rmcs_ws` 加载已编译的仿真版本。

```zsh
cd /workspaces/RMCS/rmcs_ws
source /opt/ros/jazzy/setup.zsh
source install/local_setup.zsh
source /tmp/jrmcs-mujoco-core/install/local_setup.zsh
source /tmp/jrmcs-mujoco-bridge/install/local_setup.zsh
```

## 宿主机 MuJoCo 官方窗口

在 devcontainer 的 zsh 中加载上面的环境，启动 jRMCS 闭环。`simulate.yaml` 让宿主机的 `simulate` 推进物理，`controller_list` 选择控制项。关节反馈、物理重力和力矩在本机回环地址的 UDP 17971、17972 端口交换。更新后重建 `rmcs_core` 和 `rmcs_mujoco_bridge`，并用下方脚本重新打开官方窗口，使两端使用相同的报文格式。

```zsh
export ROS_DOMAIN_ID=221 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
config="$(ros2 pkg prefix --share rmcs_mujoco_bridge)/config"
ros2 run rmcs_executor rmcs_executor --ros-args \
  --params-file "$config/position_test.yaml" \
  --params-file "$config/interactive.yaml" \
  --params-file "$config/simulate.yaml" \
  -r /robot_description:=/mujoco/robot_description -r /joint_states:=/mujoco/joint_states
```

在宿主机终端启动已经安装的官方窗口。脚本把 JGarm 转成临时 MJCF，加载力矩回调并调用原有的 `bin/simulate`。七轴按 MATLAB 的全零位启动。

```zsh
bash /sim-workspaces/jRMCS/simulation/rmcs_mujoco_bridge/run_simulate.sh \
  /sim-workspaces/mujoco-3.17.0 \
  /home/hanner/micromamba/envs/mujoco_env/bin/python
```

模型从全零位启动，初始关闭物理重力和接触。做有重力测试时，先用 **Space** 暂停，取消 **Physics → Disable Flags → Gravity** 的勾选，再恢复步进。默认重力向量为 `(0, 0, -9.81) m/s²`。若要模拟不同安装倾角，在暂停时修改 **Physics → Gravity** 的三个分量，再恢复步进。暂停期间窗口仍向 jRMCS 发送反馈，使补偿能在恢复步进前跟上重力变化。全零位的 J2 和 J4 位于限位，运行中突然开启重力可能因软限位越界使反馈失效。

纯重力悬停可用 `-p 'controller_list:=[gravity]' -p 'gripper_enabled:=false'` 启动，从静止、可承载的姿态观察角度变化。临时 MJCF 保留 URDF 的几何和惯性数据精度，两指通过高刚度齿条约束反向同步。

重力拖动测试可用 `-p 'controller_list:=[gravity, friction]'` 启用摩擦补偿。向 `arm_solver` 填入七轴辨识系数 `friction_viscous`（N·m·s/rad）和 `friction_coulomb`（N·m），两者默认零。补偿使用实测速度，`0.01 rad/s` 内为零，`friction_scale` 默认 `0.25`。仿真对比时在同一份带摩擦的 MJCF 上切换补偿项，保持外力和重力工况一致。

展开右侧 **Control**，用 `rmcs_J1`～`rmcs_J7` 设置目标角。用 `rmcs_gripper` 设置闭合比例，`0` 为全开、`1` 为全闭，中间数值保持对应开度。单个虚拟电机通过齿条约束带动两指反向同步，`ArmSolver` 根据反馈下发力矩。夹爪采用仿真专用的 10 mm 节圆半径和 0.05 N·m 力矩上限。可以在 **Joint** 查看 `joint8` 和 `joint9` 的位置，手动改动 **Joint** 会直接摆放模型。退出窗口后，在容器终端按 `Ctrl+C` 结束 jRMCS。

## MoveIt 规划并执行

使用本机官方 `simulate` 和同一个 JGarm 模型。MoveIt 通过 `FollowJointTrajectory` 下发七轴时间轨迹，夹爪通过 `GripperCommand` 下发单侧滑块位置（`0` m 为闭合，`0.032` m 为全开）。启动文件将 `joint8` 设为 `joint9` 的反向 mimic，状态反馈以实测的 `joint9` 为准。这个模式由 `TrajectoryTarget` 提供关节和夹爪目标，`ArmSolver` 计算力矩。初始为七轴全零、夹爪全开，接触和重力沿用官方窗口启动值。

重建下方的 `rmcs_core` 和仿真包后，在 devcontainer 的 zsh 中启动闭环。

```zsh
source /opt/ros/jazzy/setup.zsh
source /workspaces/RMCS/rmcs_ws/install/local_setup.zsh
source /tmp/jrmcs-mujoco-bridge/install/local_setup.zsh
export ROS_DOMAIN_ID=224 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
config="$(ros2 pkg prefix --share rmcs_mujoco_bridge)/config/moveit.yaml"
ros2 run rmcs_executor rmcs_executor --ros-args --params-file "$config" \
  -r /robot_description:=/mujoco/robot_description
```

在宿主机按上一节打开官方窗口，再用另一个宿主机终端加载 MoveIt 的 Jazzy 环境。首次运行时，可以把两个模型配置包单独构建到宿主机的临时 overlay。

```zsh
cd /sim-workspaces/jRMCS
source /opt/ros/jazzy/setup.zsh
colcon --log-base /tmp/jrmcs-moveit-host/log build \
  --base-paths rmcs_ws/src/arm_description rmcs_ws/src/arm_moveit_config \
  --build-base /tmp/jrmcs-moveit-host/build --install-base /tmp/jrmcs-moveit-host/install \
  --merge-install --symlink-install
source /tmp/jrmcs-moveit-host/install/local_setup.zsh
export ROS_DOMAIN_ID=224 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 launch arm_moveit_config mujoco.launch.py
```

RViz 中选择 `JGarm` 或 `Gripper` 规划组，再执行 `Plan & Execute`。夹爪可选 `Gripper_open` 与 `Gripper_closed`。七轴轨迹按 MuJoCo 仿真时间采样，执行时请保持物理步进。命令行验收可在启动时加 `use_rviz:=false`。

## 容器内 Qt 窗口

打开两个 devcontainer 的 zsh 终端。第一个终端加载上面的环境并启动连续仿真。`interactive.yaml` 沿用固定目标测试的模型和增益，将目标来源替换为滑条指令。

```zsh
export ROS_DOMAIN_ID=221 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
config="$(ros2 pkg prefix --share rmcs_mujoco_bridge)/config"
ros2 run rmcs_executor rmcs_executor --ros-args \
  --params-file "$config/position_test.yaml" --params-file "$config/interactive.yaml" \
  -r /robot_description:=/mujoco/robot_description -r /joint_states:=/mujoco/joint_states
```

第二个终端需要单独加载 ROS 和仿真包的环境，然后打开窗口。当前容器使用 Mesa 软件渲染。

```zsh
source /opt/ros/jazzy/setup.zsh
source /tmp/jrmcs-mujoco-bridge/install/local_setup.zsh
export ROS_DOMAIN_ID=221 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
LIBGL_ALWAYS_SOFTWARE=1 ros2 run rmcs_mujoco_bridge mujoco_viewer
```

拖动滑条改变目标角，窗口中的机械臂跟随 `/mujoco/joint_states` 的实测仿真角。连续模式从七轴零位开始，默认以 `0.5 rad/s` 限制目标变化，新目标在关节端点留出 `0.05 rad` 的余量。视口内左键旋转、右键平移、滚轮缩放。关闭窗口结束显示，在第一个终端按 `Ctrl+C` 停止仿真。

## 固定目标基准测试

在独立的 ROS 域中比较三种工况。先跑无重力 PD，再打开物理重力，最后叠加 `ArmSolver` 的重力补偿。各次使用相同的初始位姿、目标、控制增益和力矩上限。

```zsh
config="$(ros2 pkg prefix --share rmcs_mujoco_bridge)/config/position_test.yaml"
export ROS_DOMAIN_ID=221 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 run rmcs_executor rmcs_executor --ros-args --params-file "$config"
ros2 run rmcs_executor rmcs_executor --ros-args --params-file "$config" -p 'gravity:=true'
ros2 run rmcs_executor rmcs_executor --ros-args --params-file "$config" \
  -p 'gravity:=true' -p 'controller_list:=[pd, gravity]'
```

每轴报告有符号的最终位置误差 `实际角−目标角`、越过目标方向的最大超调、已施加力矩峰值，以及达到仿真力矩上限的采样比例。输出 `Position test completed` 表示仿真跑完整段。两环增益在 `ArmSolver` 构造函数内逐轴设置，仿真力矩限额在 YAML 中设置。

## 重建仿真包

MuJoCo SDK 位于宿主机 `/sim-workspaces/mujoco-3.17.0`，本次验证使用复制到容器内的 `/tmp/jrmcs-mujoco-sdk`。重建容器后，先重新挂载或复制 SDK，再将 `MUJOCO_ROOT` 设为容器内路径，该目录应包含 `include/mujoco` 和 `lib/libmujoco.so`。

在新的 zsh 终端中，从当前源码构建 `rmcs_core` 和仿真包，在 `rmcs_ws` 下执行：

```zsh
cd /workspaces/RMCS/rmcs_ws
source /opt/ros/jazzy/setup.zsh
source install/local_setup.zsh
../.script/build-rmcs --packages-select rmcs_core
source install/local_setup.zsh
colcon --log-base /tmp/jrmcs-mujoco-bridge/log build \
  --base-paths ../simulation/rmcs_mujoco_bridge \
  --build-base /tmp/jrmcs-mujoco-bridge/build \
  --install-base /tmp/jrmcs-mujoco-bridge/install \
  --merge-install \
  --cmake-args -DMUJOCO_ROOT=/tmp/jrmcs-mujoco-sdk -DCMAKE_BUILD_TYPE=Release
source /tmp/jrmcs-mujoco-bridge/install/local_setup.zsh
```

重建后使用同一终端运行前面的仿真命令，此时 `rmcs_core` 来自工作区的 `install`。

窗口使用当前 devcontainer 已安装的 Qt 5 和 X11。只需要固定目标基准测试时，可加 `-DRMCS_MUJOCO_BUILD_VIEWER=OFF` 跳过窗口编译。
