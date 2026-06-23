# Navigation Behavior Tree Project

基于 **行为树 (Behavior Tree)** 的自主导航项目，使用 **A\* (A-Star) 寻路算法**，兼容 **Groot2** 可视化编辑器。

---

## 项目结构

```
navigation_bt/
├── CMakeLists.txt                        # CMake 构建系统
├── README.md                             # 项目说明（本文件）
│
├── config/
│   └── groot2_project.xml                # Groot2 项目配置
│
├── include/
│   ├── core/
│   │   ├── types.h                       # 基础类型定义 (Point, GridCell, Path, AStarNode)
│   │   └── grid_map.h                    # 网格地图类
│   ├── algorithms/
│   │   └── astar.h                       # A* 寻路算法
│   └── bt_nodes/
│       ├── condition_nodes.h             # 条件节点 (HasTarget, HasPath, IsTargetReached...)
│       └── navigation_nodes.h            # 动作节点 (ComputePath, MoveToNextWaypoint...)
│
├── src/
│   ├── core/
│   │   └── grid_map.cpp                  # 网格地图实现
│   ├── algorithms/
│   │   └── astar.cpp                     # A* 算法实现
│   ├── bt_nodes/
│   │   ├── condition_nodes.cpp           # 条件节点实现
│   │   └── navigation_nodes.cpp          # 动作节点实现
│   └── main.cpp                          # 主演示程序
│
├── trees/
│   ├── navigation_main.xml               # 主导航行为树 (Groot2 可直接打开)
│   ├── pathfinding_subtree.xml           # 寻路子行为树
│   └── obstacle_avoidance_subtree.xml    # 避障子行为树
│
├── maps/
│   └── sample_map.txt                    # 示例网格地图 (25x15)
│
└── scripts/
    └── run_demo.sh                       # 一键编译运行脚本
```

---

## 算法: A\* (A-Star) 寻路

**A\*** 是计算机科学中最著名、使用最广泛的路径规划算法之一。

### 算法简介
- **发明者**: Peter Hart, Nils Nilsson 和 Bertram Raphael (1968)
- **发表**: IEEE Transactions on Systems Science and Cybernetics
- **原理**: `f(n) = g(n) + h(n)`
  - `g(n)`: 从起点到节点 n 的实际代价
  - `h(n)`: 从节点 n 到终点的启发式估计代价（必须可接受，不可高估）
- **时间复杂度**: O(E log V)（使用二叉堆）
- **空间复杂度**: O(V)

### 启发式函数
本项目实现了多种启发式函数：
| 函数 | 适用场景 | 公式 |
|------|---------|------|
| **Octile** | 8 方向移动（默认） | `max(dx,dy) + (√2-1)·min(dx,dy)` |
| Manhattan | 4 方向移动 | `|dx| + |dy|` |
| Euclidean | 任意方向 | `√(dx² + dy²)` |
| Chebyshev | 8 方向单位代价 | `max(|dx|, |dy|)` |

---

## 行为树节点

### 条件节点 (Condition)
| 节点 | 功能 |
|------|------|
| `HasTarget` | 检查是否设置了导航目标 |
| `HasPath` | 检查是否有剩余路径航点 |
| `IsTargetReached` | 检查是否到达目标（含距离阈值） |
| `IsPathBlocked` | 检查下一个航点是否被阻挡 |
| `IsObstacleNearby` | 扫描周围是否有障碍物 |

### 动作节点 (Action)
| 节点 | 功能 |
|------|------|
| `SetTarget` | 设置导航目标坐标 |
| `UpdateAgentPosition` | 更新智能体位置 |
| `ComputePath` | 调用 A* 计算最优路径 |
| `MoveToNextWaypoint` | 沿路径移动一个航点 |
| `MoveToTarget` | 直接向目标移动（回退策略） |
| `RotateTowardTarget` | 转向目标方向 |
| `WaitAtObstacle` | 遇到障碍时等待（指数退避） |
| `RecomputePath` | 清除并重新计算路径 |

### 行为树结构

```
Repeat (重试直到成功)
└── ReactiveSequence (每 tick 重新评估)
    ├── UpdateAgentPosition (传感器更新)
    ├── HasTarget? (有目标吗)
    └── Sequence: NavigateToTarget
        ├── ComputePath (A* 寻路)
        ├── SubTree: FollowPath (跟随路径)
        │   └── ReactiveSequence
        │       ├── HasPath? (还有航点吗)
        │       ├── IsPathBlocked? → NOT (前方畅通吗)
        │       └── Fallback (移动/处理障碍)
        │           ├── 遇障 → WaitAtObstacle → RecomputePath
        │           ├── 正常 → MoveToNextWaypoint
        │           └── 回退 → MoveToTarget
        └── IsTargetReached? (到了吗)
```

---

## 编译与运行

### 依赖
- **CMake** >= 3.16
- **C++17** 编译器 (GCC 9+, Clang 10+, MSVC 2019+)
- **BehaviorTree.CPP** (通过 CMake FetchContent 自动下载)
- **(可选) Groot2** — 行为树可视化编辑器（非编译依赖）

### 编译

```bash
cd navigation_bt
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(nproc)
```

### 运行

```bash
# 自动使用默认行为树和地图
./navigation_demo

# 指定行为树 XML 和地图文件
./navigation_demo ../trees/navigation_main.xml ../maps/sample_map.txt
```

### 一键运行

```bash
bash scripts/run_demo.sh
```

---

## 如何用 Groot2 打开

### 步骤 1: 安装 Groot2

**方法 A: 直接下载**
- 前往 [BehaviorTree.CPP Releases](https://github.com/BehaviorTree/BehaviorTree.CPP/releases)
- 下载对应平台的 Groot2 安装包

**方法 B: 从源码编译**
```bash
git clone https://github.com/BehaviorTree/Groot2.git
cd Groot2
mkdir build && cd build
cmake ..
cmake --build .
```

### 步骤 2: 打开行为树

1. **启动 Groot2**
2. **加载行为树 XML 文件**:
   - `File` → `Load Tree`
   - 选择 `trees/navigation_main.xml`
3. **Groot2 将以可视化方式展示完整的行为树结构**，包括：
   - 所有 Sequence、Fallback、Retry 控制节点
   - 所有条件和动作节点
   - 子树引用

### 步骤 3: 实时监控（可选）

1. **运行导航程序**:
   ```bash
   cd navigation_bt/build
   ./navigation_demo
   ```
   程序启动后会在端口 1667 上发布 ZMQ 数据流。

2. **在 Groot2 中连接**:
   - 菜单: `Monitor` → `Connect`
   - 地址: `tcp://localhost:1667`
   - 点击连接后即可实时观察每个节点的执行状态
   - 正在执行的节点会高亮显示
   - 节点状态（SUCCESS/FAILURE/RUNNING）用不同颜色标示

### 步骤 4: 打开项目（管理多棵行为树）

- `File` → `Open Project`
- 选择 `config/groot2_project.xml`
- 左侧面板会列出所有行为树

---

## 自定义节点注册

如果需要在 Groot2 中识别自定义节点，需要将编译的共享库加载到 Groot2：

1. 编译生成 `libnavigation_bt_nodes.so` (或 `.dll`)
2. 在 Groot2 中: `Settings` → `Custom Nodes` → 添加库路径

---

## 地图格式

地图文件格式（见 `maps/sample_map.txt`）:

```
<width> <height>
<char><char>...    # height 行, 每行 width 个字符
```

**符号说明**:
| 符号 | 含义 | 代价 |
|------|------|------|
| `.` | 可通行 | 1.0 |
| `#` | 障碍物 | 无穷 |
| `S` | 起点 | 1.0 |
| `G` | 终点 | 1.0 |
| `,` | 粗糙地形 | 2.0 |
| `~` | 水域 | 3.0 |

---

## 参考

- Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). *A Formal Basis for the Heuristic Determination of Minimum Cost Paths*. IEEE Transactions on Systems Science and Cybernetics.
- [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) — C++ 行为树框架
- [Groot2](https://github.com/BehaviorTree/Groot2) — 行为树可视化编辑器
