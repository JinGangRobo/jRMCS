# jRMCS坐标系树(Tf-Tree)

jRMCS中所有坐标与方向向量都必须以强类型标注其所属坐标系。承载这些坐标系，并在它们之间建立变换关系的，是RMCS的子模块 **fast_tf** 与由此定义的**TF-Tree*。本文说明其设计目标、定义与在RMCS中的具体拓扑。

---

## 1. 设计目标与总体结构

### 1.1 坐标系树的作用

一台机器人上同时存在多个坐标系：底盘上的‘base_link’、云台的Yaw及Pitch、相机‘camera_link’、测速模块指向'muzzle_link'，以及各惯性测量单元。控制与视觉算法需要在这些坐标系之间频繁换算坐标、方向向量与姿态。故实际tf是告诉你当前坐标系之间的关系以及在空间中的方向和指向。

那TF-Tree中是如何联系的呢，最朴素的做法是在每个用到变换的地方手写坐标变换矩阵，但相对应的是手搓难以复用，且一旦某条变换写错，错误会分散至算法内部难以定位。

ROS官方TF，虽然解决了复用问题，但与此同时，引入了另一类问题：

- TF中的坐标系以**字符串**标识（如'camera_link'），整颗树在**运行期**维护与查询；
-  字符串拼写错误，坐标系不存在、树中坐标系断链都只能在运行期暴露；
-  每次查询都伴随字符串比较和动态分配。
  
对于云台这种要求高响应、1kHz闭环的高频控制器，ROS官方TF会产生非必要的花销与不确定性，更关键的是RMCS要求坐标与方向向量以强类型携带坐标系信息，而基于字符串的运行期TF与这套强类型体系是割裂的。由此引入子模块**fast_tf**

### 1.2 fast_tf：编译器类型树

fast_tf的核心思想是：**把坐标系树的拓扑直接编码进C++类型系统。**

- 一个坐标系就是一个**类型**（'Link'）；
- 一个父子关系就是一个**模板特化**（'Joint<Child>'指定'Parent'）;
- 于是"两个坐标系之间是否存在通路""通路包含节点"，都是在**编译期**唯一确定的。
由此，运行期不再出现任何字符串、哈希表或动态分配，只有对 `std::tuple` 的 `std::get`（其下标在编译期确定）；字符串只在最后一步——向 ROS 广播 `/tf` 时——才出现一次。

所以到目前实现了

- **(1) 错误处理提前。** 坐标系拼错、路径不存在、树成环，都会导致编译失败，而不会 进入运行期。
- **(2) 每帧零查询开销。** 变换路径在编译期展开，运行期只做矩阵连乘。
- **(3) 与强类型体系统一。** 坐标系本身成为类型的一部分，统一模板。

### 1.3 架构总览
fast_tf自下而上
| 层 | 载体 | 职责 |
| --- | --- | --- |
| ① 帧标签 | `Link<T>` | 给坐标、方向向量与姿态打上所属坐标系的定义 |
| ② 边定义 | `Joint<Child>` | 声明父坐标系，并给出该边的存储形态与读写接口 |
| ③ 运行期状态 | `JointCollection` | 以 `std::tuple` 容纳全树数值，提供写入与读取 |
| ④ 查询与换算 | `lookup_transform` / `cast` | 借助最近公共祖先与矩阵连乘完成变换 |
| ⑤ 广播 | `fast_tf::rcl` | 将全树序列化为 `TransformStamped` 发布到 `/tf` |

## 2. 符号约定

### 2.1 坐标变换

```text
T_AB  :=    B 坐标系在 A 坐标系中的位姿
            即：把"B 坐标系下的量表达成 A 坐标系"的位姿变换
```

**坐标变换基本性质**

点坐标变换：
```text
p_A = T_AB · p_B
```

其中，$p_B$ 是点在 B 坐标系下的齐次坐标，$p_A$ 是同一点在 A 坐标系下的齐次坐标。若点的三维坐标分别为 $(x_A, y_A, z_A)$ 和 $(x_B, y_B, z_B)$，则：

$$p_A = \begin{bmatrix}x_A\\y_A\\z_A\\1\end{bmatrix},
\qquad
p_B =\begin{bmatrix}x_B\\y_B\\z_B\\1\end{bmatrix}
$$

则 \(T_{AB}\) 是 \(4\times 4\) 齐次变换矩阵：
\[      T_{AB} =
\begin{bmatrix} 
        R_{AB} & t_{AB}^A\\0 & 1
\end{bmatrix}
\]

---

坐标逆变换：
```text
T_BA = (T_AB)^(-1)
```
因为 \(T_{AB}\) 表示 \(B \to A\)，所以它的逆表示 \(A \to B\)。

对于齐次变换矩阵：
\[  T_{AB} =   \begin{bmatrix}  R & t\\0 & 1   \end{bmatrix}    \]
其逆为：
\[  T_{BA} =   \begin{bmatrix}  R^T & -R^T t\\0 & 1 \end{bmatrix}   \]
这里利用了旋转矩阵的正交性：
\[  R^{-1} = R^T    \]

---

复合变换
```text
T_AC = T_AB · T_BC
```
含义是：\[  C \to B \to A   \]

所以：\[p_A = T_{AB}\, p_B= T_{AB}\, T_{BC}\, p_C= T_{AC}\, p_C\]
推出    \[T_{AB} T_{BC} = T_{AC}\]

### 2.2 边的存储定义
fast_tf 中，一条父子关系（下称"边"）由 `Joint<Child>` 的一个特化描述，其中：

```text
Joint<Child>::Parent      ≡ 父坐标系
Joint<Child> 中存储的变换  ≡ T_{Parent ← Child}
```

因此 `get_transform<From, To>()` 返回的即是 `T_{From ← To}`。必须注意：变换的下标顺序与模板参数的顺序一致——`From` 在前、`To` 在后，即"由 To 到 From"。

- 所有坐标系**必须**为右手系，坐标轴方向与 ROS 相同： x 向前、y 向左、z 向上。
- 三维旋转**必须**遵循 优先顺序：四元数 ＞ 轴角 ＞ 旋转矩阵，**不允许**使用欧拉角。坐标系树中：姿态以四元数存储，关节角以轴角（`Eigen::AngleAxisd`）表示，一般变换以旋转矩阵表示。
- **必须**使用 SI 单位：长度为米、角度为弧度；浮点用 `double`，组件接口中的整数用 `int64_t`。

---

## 3. fast_tf 的类型系统
### 3.1 帧标签：Link&lt;T&gt;
详情见 [link.hpp](../../rmcs_ws/src/fast_tf/include/fast_tf/impl/link.hpp#L10-L74)。
`Link<T>` 自身不存储数据，它为所属坐标系提供三个内嵌的载荷类型：
```cpp
template <typename LinkT>
struct Link {
    using LinkType = LinkT;

    struct Position        { using LinkType = LinkT; Eigen::Vector3d    position;   };
    struct DirectionVector { using LinkType = LinkT; Eigen::Vector3d    vector;     };
    struct Rotation        { using LinkType = LinkT; Eigen::Quaterniond quaternion; };
};
```
关键在于每个载荷类型内部的 `using LinkType = LinkT;`：**值自身携带所属坐标系的信息**。`cast` 正是通过读取载荷的 `LinkType` 反推其来源坐标系。

在 `rmcs_description` 中，每个具体坐标系都是 `Link<自身>` 的派生结构，并给出广播所用的名字（见 [tf_description.hpp](../../rmcs_ws/src/rmcs_description/include/rmcs_description/tf_description.hpp#L9-L74)）：

```cpp
struct CameraLink : fast_tf::Link<CameraLink> {
    static constexpr char name[] = "camera_link";
};
```
于是 `CameraLink::Position`、`CameraLink::DirectionVector`、`CameraLink::Rotation` 就是"带 CameraLink 坐标系属性"的坐标、方向向量与姿态。

### 3.2 边：Joint&lt;Child&gt;
主模板用于表示"根"（见 [joint.hpp](../../rmcs_ws/src/fast_tf/include/fast_tf/impl/joint.hpp#L11-L16)）：
```cpp
struct Null {};

template <internal::is_link ChildT>
struct Joint { using Parent = Null; };
```
凡要进入坐标系树的坐标系，都必须给出 `Joint` 特化并指定 `Parent`。

以 `PitchLink` 为例（见 [tf_description.hpp](../../rmcs_ws/src/rmcs_description/include/rmcs_description/tf_description.hpp#L101-L110)）：
```cpp
template <>
struct fast_tf::Joint<rmcs_description::PitchLink> : fast_tf::ModificationTrackable {
    using Parent = rmcs_description::YawLink;
    void set_state(double angle) { angle_ = angle; }
    auto get_transform() const { return Eigen::AngleAxisd{angle_, Eigen::Vector3d::UnitY()}; }
private:
    double angle_;
};
```
一条边可以处于以下三种存储形态：
| 形态 | 成员 | 写入接口 | RMCS 中的实例 |
| --- | --- | --- | --- |
| 纯平移 | `Eigen::Translation3d` | `set_transform` | `GimbalCenterLink`、`MuzzleLink`、`TransmitterLink` |
| 纯旋转（姿态） | `Eigen::Quaterniond` | `set_transform` | `OdomImu`、`RawImu` |
| 完整变换 | `Eigen::Isometry3d` | `set_transform`（可只写分量） | `CameraLink`、四轮坐标系 |
| 单自由度关节 | `double`（读取时现算变换） | `set_state` | `YawLink`、`PitchLink`、`ViewerLink` |

所有边都继承 `ModificationTrackable`，其中维护一个"是否被修改"的标记（默认视为已修改）。该标记本用于增量广播，但当前 RMCS 全部采用全量广播，详见5.2。

### 3.3 全树状态：JointCollection
见 [joint_collection.hpp](../../rmcs_ws/src/fast_tf/include/fast_tf/impl/joint_collection.hpp#L55-L191)。

```cpp
template <internal::is_non_root_link... ChildLinkTs>
class JointCollection {
    using TupleT = internal::unique_tuple<Joint<ChildLinkTs>...>;
    TupleT collection_;
};
```
- **一个子坐标系对应 `tuple` 中的一个元素**，元素类型为 `Joint<Child>`；
- 读取时按 `To`（子坐标系）索引，即 `std::get<Joint<To>>(collection_)`。也就是说，**运行期只以 `To` 检索，`From` 仅参与编译期约束校验**；
- 因此同一坐标系在集合中只能出现一次（由 `unique_tuple` 保证），`contains_joint_v<To>`即"本集合是否包含以 `To` 为子节点的边"；
- 根坐标系（如 `BaseLink`）没有 `Joint` 特化，故不进入集合。

写入接口分为 `set_transform` 与 `set_state` 两组重载，其分派依据是该边的存储形态与可用接口。此外，`for_each` 会在编译期展开遍历全部边（供广播使用），`as_reference`可生成只暴露部分边的引用，用于收窄组件的访问权限。

### 3.4 编译期约束

见 [joint.hpp](../../rmcs_ws/src/fast_tf/include/fast_tf/impl/joint.hpp#L28-L71)。

`set_transform` / `set_state` 是否可用，由一组 concept 在编译期裁决：

| concept | 含义 |
| --- | --- |
| `is_non_root_link` | 有 `Joint` 特化且 `Parent != Null`，可进入 `JointCollection` |
| `has_joint<P, C>` | `Joint<C>::Parent == P`，且该边可读 |
| `has_setter_joint` | 该边提供自定义的 `set_transform` |
| `has_stateful_joint` | 该边提供 `set_state` |
| `has_getter_joint` | 该边提供 `get_transform` |
| `has_transform_joint` | 该边成员为 `Eigen::Isometry3d` |
| `has_rotation_joint` | 该边成员为四元数或轴角 |
| `has_translation_joint` | 该边成员为 `Eigen::Translation3d` |

因此"给纯平移关节设置四元数""给仅有角度的关节设置完整变换"一类的用法错误，都会在编译期被拒绝，而不会留到运行期。

## 4. fast_tf的坐标变换
同 2.1 所述，变换满足 `T_BA = (T_AB)^(-1)` 与 `T_AC = T_AB · T_BC`。本章的路径合成与坐标换算全部建立在这两条性质之上。

### 4.2 最近公共祖先与连乘
见 [cast.hpp](../../rmcs_ws/src/fast_tf/include/fast_tf/impl/cast.hpp#L10-L73)。

递归变换
```text
get_depth(Null) = 0
get_depth(T)    = get_depth(Parent(T)) + 1

get_lca(T1, T2) 按深度逐级上爬：
    T1 = T2                   : T1
    depth(T1) > depth(T2)     : get_lca(Parent(T1), T2)
    depth(T2) > depth(T1)     : get_lca(T1, Parent(T2))
    否则                      : get_lca(Parent(T1), Parent(T2))
```
由公共祖先向上连乘到目标坐标系：

```text
accumulate(Lca, Link) = accumulate(Lca, Parent(Link)) · T_{Parent(Link) ← Link} 
终止条件：Lca = Parent(Link)
result： T_{Lca ← Link}
```
即：把从 `Lca` 到 `Link` 的路径上所有边的变换按顺序相乘。

### 4.3 lookup_transform
见 [cast.hpp](../../rmcs_ws/src/fast_tf/include/fast_tf/impl/cast.hpp#L174-L189)。

```text
lookup_transform(From, To) = T_{From ← To}
    情形 a  Lca = From : accumulate(Lca, To)
    情形 b  Lca = To   : accumulate(Lca, From)^(-1)
    情形 c  其他       : accumulate(Lca, From)^(-1) · accumulate(Lca, To)
```

情形 c 的含义是：先由 `From` 退到公共祖先，再下到 `To`，即`T_{From←Lca} · T_{Lca←To} = T_{From←To}`。

例如自瞄重投影中，`fast_tf::lookup_transform<OdomImu, CameraLink>(*tf_)` 得到"相机在`OdomImu` 中的位姿"（见 [auto_aim.cpp](../../rmcs_ws/src/rmcs_core/src/referee/app/ui/auto_aim.cpp#L185)）。

### 4.4 cast

见 [cast.hpp](../../rmcs_ws/src/fast_tf/include/fast_tf/impl/cast.hpp#L102-L172)。

```text
cast<To>(From::Position{p})
    情形 a : accumulate(Lca, To)^(-1) · p
    情形 b : accumulate(Lca, From)     · p
    情形 c : accumulate(Lca, To)^(-1) · accumulate(Lca, From) · p

cast<To>(From::DirectionVector{v}) 与 cast<To>(From::Rotation{q}) 与上式相同，但只取线性部分 linear(T)，即丢弃平移
```
理由：方向向量与姿态只表示朝向，不应受坐标系原点平移的影响，故只施加变换的旋转部分。

例如云台解算中，`fast_tf::cast<PitchLink>(odom_dir, *tf_)` 把 `OdomImu` 下的目标方向向量转换到 `PitchLink`（见 [two_axis_gimbal_solver.hpp](../../rmcs_ws/src/rmcs_core/src/controller/gimbal/two_axis_gimbal_solver.hpp#L49-L56)）。

## 5. 写入与广播
### 5.1 set_transform 与 set_state
| 接口 | 含义 | 适用边 |
| --- | --- | --- |
| `set_transform<From, To>(x)` | 写入整条边（完整位姿，或该边的平移/旋转分量） | 静态标定、IMU 姿态 |
| `set_state<From, To>(q)` | 写入该边的单个自由度（关节角） | `YawLink`、`PitchLink`、`ViewerLink`、四轮坐标 |

两者的共同点是：写入后都会将该边标记为"已修改"。对于存储为 `Eigen::Isometry3d` 的边，`set_transform` 还允许只传入平移或旋转分量，分别只修改对应的部分（见[joint_collection.hpp](../../rmcs_ws/src/fast_tf/include/fast_tf/impl/joint_collection.hpp#L83-L139)）。

### 5.2 从类型树到 ROS /tf
见 [rcl.hpp](../../rmcs_ws/src/fast_tf/include/fast_tf/rcl.hpp#L13-L101)。
- `fast_tf::rcl::Node` 为单例，内部持有一个 `tf2_ros::TransformBroadcaster`；
- `broadcast<From, To>(collections...)` 先由 `get_transform` 取得 `T_{From ← To}`，再按该边的存储形态拆出平移与旋转，组装为 `TransformStamped`：

```text
header.frame_id = From::name
child_frame_id  = To::name
transform       = T_{From ← To}
header.stamp    = 节点时钟的当前时刻
```
- `broadcast_all(collection)`：在编译期展开遍历集合中的每一条边，逐条广播；
- `broadcast_all_modified(collection)`：只广播"已修改"的边。
  
RMCS 中所有写入者都使用 `broadcast_all`（全量广播），未使用 `broadcast_all_modified`；因此 3.2 中提到的"已修改"标记目前未被利用。另有一个独立的转发组件[tf_broadcaster.cpp](../../rmcs_ws/src/rmcs_core/src/broadcaster/tf_broadcaster.cpp#L22-L44)，在 `before_updating` 中全量广播一次，并在 `update` 中以 50 ms 为周期再次广播，为 `/tf`提供稳定的发布节奏。

---

## 6. RMCS 的坐标系树

### 6.1 拓扑

`rmcs_description` 中，坐标系树的拓扑如下。缩进表示父子关系，括号内为该边的形态，
"←" 侧标注写入者：

```text
base_link                                       （根；无 Joint 特化）
├── gimbal_center_link                          Translation3d
│   └── yaw_link                                Rz，下板 yaw 关节角
│       └── pitch_link                          Ry，上板 pitch 关节角
│           ├── muzzle_link                     Translation3d
│           ├── transmitter_link                Translation3d
│           ├── camera_link                     Isometry3d
│           ├── odom_imu                        Quaterniond
│           └── viewer_link                     Ry（仅 Tf / HeroTf）
├── raw_imu                                     Quaterniond
├── left_front_wheel_link                       Isometry3d（轮角）
├── left_back_wheel_link                        Isometry3d（轮角）
├── right_back_wheel_link                       Isometry3d（轮角）
└── right_front_wheel_link                      Isometry3d（轮角）
```

注意：`base_link` 是整棵树的根，它没有任何 `Joint` 特化，因而也不出现在`JointCollection` 中。

### 6.2 各边的类型与写入者

| From | To | 边类型 | 写入者 | 时机 |
| --- | --- | --- | --- | --- |
| `PitchLink` | `CameraLink` | `Isometry3d` | 硬件组件 | 构造期静态标定 |
| `PitchLink` | `MuzzleLink` | `Translation3d` | 硬件组件 | 构造期静态标定 |
| `PitchLink` | `OdomImu` | `Quaterniond` | 上板 | 每周期 |
| `BaseLink` | `RawImu` | `Quaterniond` | 上板 | 每周期 |
| `GimbalCenterLink` | `YawLink` | 关节角（绕 z） | 下板 | 每周期 |
| `YawLink` | `PitchLink` | 关节角（绕 y） | 上板 | 每周期 |

### 6.3 典型数据流

以双 yaw 步兵为例，串联一次坐标系树的数据流。上板负责上 yaw、pitch 与 IMU，下板负责下 yaw 和底盘；两板共同操作同一棵坐标系树。

**数据结构。** `DualInfantry` 持有一个 `OutputInterface<rmcs_description::Tf> tf_`，上板与下板各自持有该对象的引用（见 [dual_infantry.cpp](../../rmcs_ws/src/rmcs_core/src/hardware/dual_infantry.cpp)）。两板操作的是同一个 `tf_` 实例，而不是各自维护半棵树。

**1. 构造期：声明输出并写入静态标定边。**

```cpp
register_output("/tf", tf_);
tf_->set_transform<PitchLink, CameraLink>(Eigen::Translation3d{0.084, 0.0, 0.048});
tf_->set_transform<PitchLink, MuzzleLink>(Eigen::Translation3d{0.0, 0.0, 0.0});
```

这两条是与电机状态无关的固定安装关系，只需写入一次。

**2. 每周期 `DualInfantry::update()` 依次调用上板与下板更新。**

**2.1 上板（`TopBoard::update`）：**

1. 读取 IMU 快照，更新上 yaw 与 pitch 电机状态；
2. 若有 IMU 快照，写入 `OdomImu` 姿态：

```cpp
tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
    snapshot->orientation.conjugate());
```

3. 写入 `PitchLink` 关节角：

```cpp
tf_->set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(
    gimbal_pitch_motor_.angle());
```

4. **全量广播**：`fast_tf::rcl::broadcast_all(*tf_);`
5. 若有 IMU 快照，写入 `RawImu` 姿态：

```cpp
tf_->set_transform<rmcs_description::BaseLink, rmcs_description::RawImu>(
    snapshot->orientation);
```

注意上板的广播发生在写入 `RawImu` 之前，因此上板这次广播中的 `RawImu` 是此前保存的值。随后下板还会在同一周期进行一次全量广播；若上板本周期取得了 IMU 快照，下板的广播就会包含刚写入的 `RawImu` 新值。

**2.2 下板（`BottomBoard::update`）：**

1. 更新下 yaw 电机状态；
2. 写入 `YawLink` 关节角：

```cpp
tf_->set_state<rmcs_description::GimbalCenterLink, rmcs_description::YawLink>(
    gimbal_bottom_yaw_motor_.angle());
```

3. **全量广播**：`fast_tf::rcl::broadcast_all(*tf_);`
4. 随后更新底盘四个轮电机与超电状态（不写入坐标系树）。

**3. 结果。**

每个周期会调用两次 `broadcast_all`，但两次都作用于同一棵树。下板广播时，树中的 `PitchLink` 关节角与 `OdomImu` 姿态沿用上板刚写入的值；若上板本周期有 IMU 快照，`RawImu` 也已在上板广播后更新。因此两板的分工是「各自写入自己负责的边，各自触发一次全量广播」，而非各自维护半棵树。`fast_tf::rcl::Node` 是单例，内部持有同一个 `tf2_ros::TransformBroadcaster`，两次广播最终都发布到 `/tf`。

每个关节角对应一条边，其含义始终是 $T_{Parent \leftarrow Child}$，例如：

$$
T_{yaw\_link \leftarrow pitch\_link} = R_y(\theta_{pitch}),
\qquad
T_{gimbal\_center\_link \leftarrow yaw\_link} = R_z(\theta_{yaw})
$$


## References

- 《RMCS 思想和架构 v25.8.1》：
    4.2.2 单位与类型、
    4.2.3 右手系、
    4.2.4 坐标轴方向、 
    4.2.5 坐标和方向向量、
    4.2.7 三维旋转、
    4.3.1 组件、
    5.1.3 云台控制
- fast_tf：
  [link.hpp](../../rmcs_ws/src/fast_tf/include/fast_tf/impl/link.hpp)、
  [joint.hpp](../../rmcs_ws/src/fast_tf/include/fast_tf/impl/joint.hpp)、
  [joint_collection.hpp](../../rmcs_ws/src/fast_tf/include/fast_tf/impl/joint_collection.hpp)、
  [cast.hpp](../../rmcs_ws/src/fast_tf/include/fast_tf/impl/cast.hpp)、
  [rcl.hpp](../../rmcs_ws/src/fast_tf/include/fast_tf/rcl.hpp)
- RMCS 坐标系定义：
  [tf_description.hpp](../../rmcs_ws/src/rmcs_description/include/rmcs_description/tf_description.hpp)、
  [sentry_description.hpp](../../rmcs_ws/src/rmcs_description/include/rmcs_description/sentry_description.hpp)