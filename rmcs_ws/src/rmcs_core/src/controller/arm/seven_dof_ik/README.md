# 七轴逆解算法目录


## 文件职责

| 当前文件 | 原文件 | 职责 |
| --- | --- | --- |
| `seven_dof_ik.hpp` | `myikine7R.hpp` | 基础类型、FK、SEW 闭式候选、限位筛选与配置 |
| `hybrid_ik.hpp` | `myikine7RHybrid.hpp` | SEW 流形投影、电机命令曲线投影、Hybrid 调度 |
| `dls_ik.hpp` | `myikine7RDls.hpp` | 固定内存加权 DLS 局部迭代 |
| `adaptive_hybrid_ik.hpp` | `myikine7RAdaptive.hpp` | 按目标变化量互斥选择 DLS / Hybrid |

`adaptive_hybrid_ik.hpp` 是 `ArmSolver` 的唯一入口，内部 include 关系为：

```text
adaptive_hybrid_ik.hpp
  ├─ dls_ik.hpp
  │    └─ seven_dof_ik.hpp
  └─ hybrid_ik.hpp
       └─ seven_dof_ik.hpp
```

