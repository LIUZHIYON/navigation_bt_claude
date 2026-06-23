/**
 * @file navigation_nodes.cpp
 * @brief Implementation of navigation action nodes.
 */

#include "bt_nodes/navigation_nodes.h"
#include "bt_nodes/condition_nodes.h"

#include <cmath>
#include <iostream>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace nav_bt {

// ============================================================
// SetTarget
// ============================================================
SetTarget::SetTarget(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList SetTarget::providedPorts() {
    return {
        BT::InputPort<int>("x", "Target X coordinate"),
        BT::InputPort<int>("y", "Target Y coordinate")
    };
}

BT::NodeStatus SetTarget::tick() {
    auto xOpt = getInput<int>("x");
    auto yOpt = getInput<int>("y");
    int x = xOpt.value_or(0);
    int y = yOpt.value_or(0);
    config().blackboard->set("target_x", x);
    config().blackboard->set("target_y", y);
    config().blackboard->set("target_set", true);
    std::cout << "[SetTarget] target=(" << x << "," << y << ")\n";
    return BT::NodeStatus::SUCCESS;
}

// ============================================================
// UpdateAgentPosition
// ============================================================
UpdateAgentPosition::UpdateAgentPosition(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList UpdateAgentPosition::providedPorts() {
    return {
        BT::InputPort<int>("x", 0, "Current X"),
        BT::InputPort<int>("y", 0, "Current Y")
    };
}

BT::NodeStatus UpdateAgentPosition::tick() {
    // Already initialized from main — just verify
    int ax = 0, ay = 0;
    try { ax = config().blackboard->get<int>("agent_x"); } catch (...) {}
    try { ay = config().blackboard->get<int>("agent_y"); } catch (...) {}
    return BT::NodeStatus::SUCCESS;
}

// ============================================================
// ComputePath — runs A* algorithm
// ============================================================
ComputePath::ComputePath(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList ComputePath::providedPorts() {
    return {
        BT::InputPort<std::string>("algorithm", "astar", "Algorithm name")
    };
}

BT::NodeStatus ComputePath::tick() {
    int ax, ay, tx, ty;
    try {
        ax = config().blackboard->get<int>("agent_x");
        ay = config().blackboard->get<int>("agent_y");
        tx = config().blackboard->get<int>("target_x");
        ty = config().blackboard->get<int>("target_y");
    } catch (const std::exception& e) {
        std::cerr << "[ComputePath] missing data: " << e.what() << "\n";
        return BT::NodeStatus::FAILURE;
    }

    Point start(ax, ay), goal(tx, ty);

    AStarPathfinder localFinder;
    AStarPathfinder* finder = pathfinder_ ? pathfinder_ : &localFinder;

    GridMap* map = nullptr;
    try { map = config().blackboard->get<GridMap*>("grid_map"); } catch (...) {}
    if (map) finder->setMap(map);

    Path path;
    AStarStats stats;
    if (!finder->findPath(start, goal, path, &stats)) {
        config().blackboard->set("path_found", false);
        std::cerr << "[ComputePath] No path! explored=" << stats.nodesExplored << "\n";
        return BT::NodeStatus::FAILURE;
    }

    config().blackboard->set("path", path);
    config().blackboard->set("path_index", 0);
    config().blackboard->set("path_found", true);

    std::cout << "[ComputePath] path found! len=" << stats.pathLength
              << " cost=" << stats.totalCost << " nodes=" << stats.nodesExplored
              << " time=" << stats.searchTimeMs << "ms\n";
    return BT::NodeStatus::SUCCESS;
}

// ============================================================
// MoveToNextWaypoint — advance one step along the path
// ============================================================
MoveToNextWaypoint::MoveToNextWaypoint(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList MoveToNextWaypoint::providedPorts() {
    return { BT::InputPort<double>("speed", 1.0, "Cells per tick") };
}

BT::NodeStatus MoveToNextWaypoint::onStart() {
    speed_ = getInput<double>("speed").value_or(1.0);
    progress_ = 0.0;
    return onRunning(); // process immediately
}

BT::NodeStatus MoveToNextWaypoint::onRunning() {
    Path path;
    int index = 0;
    try {
        path  = config().blackboard->get<Path>("path");
        index = config().blackboard->get<int>("path_index");
    } catch (...) {
        std::cerr << "[MoveToNextWaypoint] no path data\n";
        return BT::NodeStatus::FAILURE;
    }

    if (index < 0 || static_cast<size_t>(index) >= path.size()) {
        return BT::NodeStatus::SUCCESS;  // reached end of path
    }

    const Point& wp = path[static_cast<size_t>(index)];

    // Move agent to waypoint
    config().blackboard->set("agent_x", wp.x);
    config().blackboard->set("agent_y", wp.y);
    config().blackboard->set("path_index", index + 1);

    int left = static_cast<int>(path.size()) - index - 1;
    std::cout << "[MoveToNextWaypoint] step #" << index << " → " << wp
              << "  [" << left << " remaining]\n";
    return BT::NodeStatus::SUCCESS;
}

void MoveToNextWaypoint::onHalted() { progress_ = 0.0; }

// ============================================================
// MoveToTarget — direct movement toward target (fallback)
// ============================================================
MoveToTarget::MoveToTarget(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList MoveToTarget::providedPorts() { return {}; }

BT::NodeStatus MoveToTarget::onStart() {
    return onRunning();
}

BT::NodeStatus MoveToTarget::onRunning() {
    int cx, cy, gx, gy;
    try {
        cx = config().blackboard->get<int>("agent_x");
        cy = config().blackboard->get<int>("agent_y");
        gx = config().blackboard->get<int>("target_x");
        gy = config().blackboard->get<int>("target_y");
    } catch (...) { return BT::NodeStatus::FAILURE; }

    if (cx == gx && cy == gy) {
        std::cout << "[MoveToTarget] already at target\n";
        return BT::NodeStatus::SUCCESS;
    }

    int dx = (gx > cx) ? 1 : (gx < cx) ? -1 : 0;
    int dy = (gy > cy) ? 1 : (gy < cy) ? -1 : 0;
    int nx = cx + dx, ny = cy + dy;

    GridMap* map = nullptr;
    try { map = config().blackboard->get<GridMap*>("grid_map"); } catch (...) {}

    if (map && !map->isTraversable(nx, ny)) {
        bool moved = false;
        for (const auto& dir : kCardinalDirs) {
            int ax2 = cx + dir[0], ay2 = cy + dir[1];
            if (map->isTraversable(ax2, ay2) &&
                (std::abs(ax2-gx)+std::abs(ay2-gy)) < (std::abs(cx-gx)+std::abs(cy-gy))) {
                nx = ax2; ny = ay2; moved = true; break;
            }
        }
        if (!moved) { std::cerr << "[MoveToTarget] stuck\n"; return BT::NodeStatus::FAILURE; }
    }

    config().blackboard->set("agent_x", nx);
    config().blackboard->set("agent_y", ny);
    std::cout << "[MoveToTarget] step → (" << nx << "," << ny << ")\n";
    return (nx == gx && ny == gy) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

void MoveToTarget::onHalted() { std::cout << "[MoveToTarget] halted\n"; }

// ============================================================
// RotateTowardTarget
// ============================================================
RotateTowardTarget::RotateTowardTarget(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList RotateTowardTarget::providedPorts() { return {}; }

BT::NodeStatus RotateTowardTarget::onStart() { return onRunning(); }

BT::NodeStatus RotateTowardTarget::onRunning() {
    int ax, ay, tx, ty;
    try {
        ax = config().blackboard->get<int>("agent_x");
        ay = config().blackboard->get<int>("agent_y");
        tx = config().blackboard->get<int>("target_x");
        ty = config().blackboard->get<int>("target_y");
    } catch (...) { return BT::NodeStatus::FAILURE; }

    double cur = 0.0;
    try { cur = config().blackboard->get<double>("agent_heading"); } catch (...) {}

    double desired = std::atan2((double)(ty-ay), (double)(tx-ax));
    double diff = desired - cur;
    while (diff >  M_PI) diff -= 2*M_PI;
    while (diff < -M_PI) diff += 2*M_PI;

    if (std::abs(diff) < 0.05) {
        config().blackboard->set("agent_heading", desired);
        return BT::NodeStatus::SUCCESS;
    }
    double step = std::clamp(diff, -0.3, 0.3);
    config().blackboard->set("agent_heading", cur + step);
    return BT::NodeStatus::RUNNING;
}

void RotateTowardTarget::onHalted() {}

// ============================================================
// WaitAtObstacle — exponential backoff
// ============================================================
WaitAtObstacle::WaitAtObstacle(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList WaitAtObstacle::providedPorts() {
    return { BT::InputPort<int>("wait_max", 5, "Max wait attempts") };
}

BT::NodeStatus WaitAtObstacle::onStart() {
    int count = 0;
    try { count = config().blackboard->get<int>("wait_count"); } catch (...) {}

    waitTicks_ = 1 << count;  // 1, 2, 4, 8, ...
    maxWaitTicks_ = getInput<int>("wait_max").value_or(5);

    if (count >= maxWaitTicks_) {
        std::cerr << "[WaitAtObstacle] max attempts\n";
        config().blackboard->set("wait_count", 0);
        return BT::NodeStatus::FAILURE;
    }
    config().blackboard->set("wait_count", count + 1);
    std::cout << "[WaitAtObstacle] wait " << waitTicks_ << "t (attempt "
              << count+1 << "/" << maxWaitTicks_ << ")\n";
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitAtObstacle::onRunning() {
    if (waitTicks_ > 0) { --waitTicks_; return BT::NodeStatus::RUNNING; }
    std::cout << "[WaitAtObstacle] done\n";
    return BT::NodeStatus::SUCCESS;
}

void WaitAtObstacle::onHalted() { waitTicks_ = 0; }

// ============================================================
// RecomputePath — clear and recompute A* path
// ============================================================
RecomputePath::RecomputePath(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList RecomputePath::providedPorts() { return {}; }

BT::NodeStatus RecomputePath::tick() {
    int ax, ay, tx, ty;
    try {
        ax = config().blackboard->get<int>("agent_x");
        ay = config().blackboard->get<int>("agent_y");
        tx = config().blackboard->get<int>("target_x");
        ty = config().blackboard->get<int>("target_y");
    } catch (const std::exception& e) {
        std::cerr << "[RecomputePath] " << e.what() << "\n";
        return BT::NodeStatus::FAILURE;
    }

    Point start(ax, ay), goal(tx, ty);

    AStarPathfinder localFinder;
    AStarPathfinder* finder = pathfinder_ ? pathfinder_ : &localFinder;

    GridMap* map = nullptr;
    try { map = config().blackboard->get<GridMap*>("grid_map"); } catch (...) {}
    if (map) finder->setMap(map);

    Path path;
    if (!finder->findPath(start, goal, path)) {
        std::cerr << "[RecomputePath] no path found\n";
        return BT::NodeStatus::FAILURE;
    }

    config().blackboard->set("path", path);
    config().blackboard->set("path_index", 0);
    config().blackboard->set("path_found", true);
    std::cout << "[RecomputePath] new path len=" << path.size() << "\n";
    return BT::NodeStatus::SUCCESS;
}

// ============================================================
// Factory registration
// ============================================================
void registerNavigationNodes(BT::BehaviorTreeFactory& factory,
                              AStarPathfinder* pathfinder,
                              GridMap* map) {
    factory.registerNodeType<SetTarget>("SetTarget");
    factory.registerNodeType<UpdateAgentPosition>("UpdateAgentPosition");
    factory.registerNodeType<ComputePath>("ComputePath");
    factory.registerNodeType<MoveToNextWaypoint>("MoveToNextWaypoint");
    factory.registerNodeType<MoveToTarget>("MoveToTarget");
    factory.registerNodeType<RotateTowardTarget>("RotateTowardTarget");
    factory.registerNodeType<WaitAtObstacle>("WaitAtObstacle");
    factory.registerNodeType<RecomputePath>("RecomputePath");

    factory.registerNodeType<HasTarget>("HasTarget");
    factory.registerNodeType<HasPath>("HasPath");
    factory.registerNodeType<IsTargetReached>("IsTargetReached");
    factory.registerNodeType<IsPathBlocked>("IsPathBlocked");
    factory.registerNodeType<IsObstacleNearby>("IsObstacleNearby");

    std::cout << "[RegisterNodes] 13 nodes registered\n";
}

}  // namespace nav_bt
