<p align="center">
  <img src="https://img.shields.io/badge/C%2B%2B-17-blue?logo=c%2B%2B" alt="C++17">
  <img src="https://img.shields.io/badge/CMake-3.16%2B-green?logo=cmake" alt="CMake">
  <img src="https://img.shields.io/badge/Algorithm-A*%20Pathfinding-orange" alt="A*">
  <img src="https://img.shields.io/badge/Editor-Groot2-9cf?logo=trello" alt="Groot2">
  <img src="https://img.shields.io/badge/License-MIT-lightgrey" alt="License">
</p>

<h1 align="center">🧭 Navigation Behavior Tree</h1>

<p align="center">
  <b>Autonomous Navigation using A* Pathfinding &amp; Behavior Trees</b><br>
  Groot2 Visual Editor Compatible | BehaviorTree.CPP v4 | C++17
</p>

---

## 📖 Overview 项目简介

A complete **autonomous navigation system** built with **Behavior Trees** and the classic **A\* (A-Star) pathfinding algorithm**. The behavior tree logic is defined in standard XML format, fully compatible with **Groot2** — the official visual editor for BehaviorTree.CPP.

> **核心算法**: A\* 寻路（1968，Hart, Nilsson & Raphael）— `f(n) = g(n) + h(n)`
>
> **行为树框架**: BehaviorTree.CPP v4.6 — 业界最流行的 C++ 行为树库
>
> **可视化工具**: Groot2 — 加载 XML 直接查看/编辑行为树

---

## 🚀 Quick Start 快速开始

### Prerequisites 环境要求

| 工具 | 版本要求 | 说明 |
|------|---------|------|
| CMake | ≥ 3.16 | 构建系统 |
| C++ Compiler | GCC 9+ / Clang 10+ / MSVC 2019+ | 需支持 C++17 |
| Groot2 | latest | 可视化编辑器（可选，仅用于查看行为树）|

> ⚠️ **不需要手动安装任何依赖库** — CMake 会自动从 GitHub 下载 BehaviorTree.CPP。

### 3 Steps to Run 三步运行

```bash
# 1️⃣ Clone this repo
git clone https://github.com/LIUZHIYON/navigation_bt_claude.git
cd navigation_bt_claude

# 2️⃣ Build (CMake auto-downloads all dependencies)
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(nproc)   # Linux/macOS
# cmake --build .             # Windows (MSVC)

# 3️⃣ Run the demo
./navigation_demo
```

### One-liner 一行搞定

```bash
# Or use the convenience script:
bash scripts/run_demo.sh
```

> 🎯 The demo will load a 25×15 grid map, compute an A\* path, and execute the behavior tree tick-by-tick. Output prints the map with real-time path overlay.

### Open in Groot2 用 Groot2 可视化

```bash
# 1. Download & install Groot2 from:
#    https://github.com/BehaviorTree/Groot2/releases

# 2. Launch Groot2

# 3. File → Load Tree → select:
#    trees/navigation_main.xml

# 4. (Optional) To see live execution:
#    First run:  ./navigation_demo
#    Then in Groot2: Monitor → Connect → tcp://localhost:1667
```

---

## 🗺️ Project Structure 项目结构

```
navigation_bt_claude/
│
├── include/                          # 头文件
│   ├── core/
│   │   ├── types.h                   # 基础类型 (Point, GridCell, AStarNode...)
│   │   └── grid_map.h                # 网格地图
│   ├── algorithms/
│   │   └── astar.h                   # A* 寻路 (5种启发式函数)
│   └── bt_nodes/
│       ├── condition_nodes.h         # 5 个条件节点
│       └── navigation_nodes.h        # 8 个动作节点
│
├── src/                              # 源文件实现
│   ├── core/grid_map.cpp
│   ├── algorithms/astar.cpp
│   ├── bt_nodes/{condition,navigation}_nodes.cpp
│   └── main.cpp                      # 主演示程序 + ZMQ 实时监控
│
├── trees/                            # 🌳 行为树 XML (Groot2 打开这些!)
│   ├── navigation_main.xml           # 主导航行为树 ⭐
│   ├── pathfinding_subtree.xml       # 寻路子行为树
│   └── obstacle_avoidance_subtree.xml # 避障子行为树
│
├── maps/
│   └── sample_map.txt                # 示例 25×15 网格地图
│
├── config/
│   └── groot2_project.xml            # Groot2 项目配置
│
├── scripts/
│   └── run_demo.sh                   # 一键编译运行脚本
│
├── CMakeLists.txt                    # CMake 构建文件
└── README.md
```

---

## 🧠 A\* Algorithm 算法详解

**A\*** (A-Star) is the most famous pathfinding algorithm in computer science.

| Property | Detail |
|----------|--------|
| **Paper** | Hart, Nilsson & Raphael (1968), *IEEE Trans. Systems Science and Cybernetics* |
| **Formula** | `f(n) = g(n) + h(n)` — actual cost + heuristic estimate |
| **Complexity** | O(E log V) with binary heap |
| **Optimality** | Guaranteed when heuristic is **admissible** (never overestimates) |

### Heuristic Functions 启发式函数

| Function | Movement | Formula |
|----------|----------|---------|
| **Octile** ⭐ | 8-directional (default) | `max(dx,dy) + (√2−1)·min(dx,dy)` |
| Manhattan | 4-directional | `|dx| + |dy|` |
| Euclidean | Continuous | `√(dx² + dy²)` |
| Chebyshev | 8-dir (unit cost) | `max(|dx|, |dy|)` |
| Diagonal | 8-dir (simplified) | `dx+dy + (√2−2)·min(dx,dy)` |

---

## 🌲 Behavior Tree Design 行为树设计

### Custom Nodes 自定义节点

| Type | Node | Function |
|------|------|----------|
| 🔵 Condition | `HasTarget` | 检查是否设定了导航目标 |
| 🔵 Condition | `HasPath` | 检查是否还有剩余航点 |
| 🔵 Condition | `IsTargetReached` | 检查是否到达目标 |
| 🔵 Condition | `IsPathBlocked` | 检查前方是否被阻挡 |
| 🔵 Condition | `IsObstacleNearby` | 扫描周围是否有障碍物 |
| 🟢 Action | `ComputePath` | 调用 A\* 计算最优路径 |
| 🟢 Action | `MoveToNextWaypoint` | 沿路径移动到下一个航点 |
| 🟢 Action | `MoveToTarget` | 直接向目标移动（回退策略）|
| 🟢 Action | `RotateTowardTarget` | 转向目标方向 |
| 🟢 Action | `WaitAtObstacle` | 指数退避等待 |
| 🟢 Action | `RecomputePath` | 清除并重新计算路径 |
| 🟢 Action | `SetTarget` | 设置导航目标坐标 |
| 🟢 Action | `UpdateAgentPosition` | 更新智能体位置 |

### Main Tree Logic 主行为树逻辑

```
RepeatUntilSuccess (重试直到成功)
└── ReactiveSequence (每 tick 重新评估)
    ├── UpdateAgentPosition       # 传感器/位置更新
    ├── HasTarget?                # 有导航目标吗？
    └── Sequence: NavigateToTarget
        ├── ComputePath           # 🧠 A* 计算最优路径
        ├── SubTree: FollowPath   # 🚶 跟随航点 + 避障
        └── IsTargetReached?      # ✅ 到达确认
```

### FollowPath Subtree 路径跟随子树

```
ReactiveSequence
├── HasPath?                     # 还有航点吗？
├── IsPathBlocked? → NOT         # 前方畅通吗？
└── Fallback (选择器)
    ├── 遇障 → WaitAtObstacle → RecomputePath   # 躲避障碍
    ├── 正常 → MoveToNextWaypoint                # 逐步移动
    └── 回退 → MoveToTarget                      # 直接移动
```

---

## 🎮 Groot2 Usage Groot2 使用指南

### Load a Tree 加载行为树
1. Launch **Groot2**
2. **File → Load Tree** → select `trees/navigation_main.xml`
3. The tree renders as an interactive node graph

### Live Monitor 实时监控
1. Run the demo: `./navigation_demo`
2. In Groot2: **Monitor → Connect** → `tcp://localhost:1667`
3. Watch nodes highlight in real-time as the tree executes:
   - 🟡 Yellow = RUNNING
   - 🟢 Green = SUCCESS
   - 🔴 Red = FAILURE

### Open Project 打开项目
- **File → Open Project** → `config/groot2_project.xml`
- All 3 behavior trees appear in the sidebar

---

## 🗺️ Map Format 地图格式

Create custom maps in `maps/`:

```
<width> <height>
<char><char>...     # height rows, each exactly width chars
```

| Char | Meaning | Cost |
|------|---------|------|
| `.` | Free traversable terrain | 1.0 |
| `#` / `@` | Obstacle (blocked) | ∞ |
| `S` | Start position | 1.0 |
| `G` | Goal position | 1.0 |
| `,` | Rough terrain | 2.0 |
| `~` | Water | 3.0 |

---

## 📚 References 参考

- **Hart, P. E., Nilsson, N. J., & Raphael, B.** (1968). *A Formal Basis for the Heuristic Determination of Minimum Cost Paths*. IEEE Transactions on Systems Science and Cybernetics, 4(2), 100–107.
- [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) — C++ Behavior Tree framework (v4)
- [Groot2](https://github.com/BehaviorTree/Groot2) — Behavior Tree visual editor

---

## 📄 License

MIT License — feel free to use, modify, and distribute.

---

<p align="center">
  <sub>Built with ❤️ by Claude Code | A* + Behavior Trees = Intelligent Navigation</sub>
</p>
