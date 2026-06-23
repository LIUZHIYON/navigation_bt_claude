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

    std::cout << "[SetTarget] Navigation target set to (" << x << ", " << y << ")\n";
    return BT::NodeStatus::SUCCESS;
}

// ============================================================
// UpdateAgentPosition
// ============================================================
UpdateAgentPosition::UpdateAgentPosition(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList UpdateAgentPosition::providedPorts() {
    return {
        BT::InputPort<int>("x", 0, "Current X (if not from blackboard)"),
        BT::InputPort<int>("y", 0, "Current Y (if not from blackboard)")
    };
}

BT::NodeStatus UpdateAgentPosition::tick() {
    // In a real system, read from sensors/odometry
    // For demo: read from blackboard if already set
    int ax = 0, ay = 0;
    try {
        ax = config().blackboard->get<int>("agent_x");
        ay = config().blackboard->get<int>("agent_y");
    } catch (...) {
        // First initialization: use port values
        auto xOpt = getInput<int>("x");
        auto yOpt = getInput<int>("y");
        config().blackboard->set("agent_x", xOpt.value_or(0));
        config().blackboard->set("agent_y", yOpt.value_or(0));
    }
    return BT::NodeStatus::SUCCESS;
}

// ============================================================
// ComputePath
// ============================================================
ComputePath::ComputePath(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList ComputePath::providedPorts() {
    return {
        BT::InputPort<std::string>("algorithm", "astar", "Pathfinding algorithm to use")
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
        std::cerr << "[ComputePath] ERROR: Missing data: " << e.what() << "\n";
        return BT::NodeStatus::FAILURE;
    }

    Point start(ax, ay);
    Point goal(tx, ty);

    // Use the provided pathfinder or create a temporary one
    AStarPathfinder localFinder;
    AStarPathfinder* finder = pathfinder_ ? pathfinder_ : &localFinder;

    GridMap* map = nullptr;
    try {
        map = config().blackboard->get<GridMap*>("grid_map");
    } catch (...) {}

    if (map) {
        finder->setMap(map);
    }

    std::cout << "[ComputePath] Computing A* path from " << start
              << " to " << goal << "...\n";

    Path path;
    AStarStats stats;
    bool found = finder->findPath(start, goal, path, &stats);

    if (found) {
        config().blackboard->set("path", path);
        config().blackboard->set("path_index", 0);
        config().blackboard->set("path_found", true);
        config().blackboard->set("astar_stats", stats);

        std::cout << "[ComputePath] Path found! Length=" << stats.pathLength
                  << ", Cost=" << stats.totalCost
                  << ", Explored=" << stats.nodesExplored
                  << ", Time=" << stats.searchTimeMs << "ms\n";
        return BT::NodeStatus::SUCCESS;
    } else {
        config().blackboard->set("path_found", false);
        std::cerr << "[ComputePath] No path found! Explored "
                  << stats.nodesExplored << " nodes.\n";
        return BT::NodeStatus::FAILURE;
    }
}

// ============================================================
// MoveToNextWaypoint
// ============================================================
MoveToNextWaypoint::MoveToNextWaypoint(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList MoveToNextWaypoint::providedPorts() {
    return {
        BT::InputPort<double>("speed", 1.0, "Movement speed (cells per tick)")
    };
}

BT::NodeStatus MoveToNextWaypoint::onStart() {
    auto speedOpt = getInput<double>("speed");
    speed_ = speedOpt.value_or(1.0);
    progress_ = 0.0;
    std::cout << "[MoveToNextWaypoint] Starting movement...\n";
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveToNextWaypoint::onRunning() {
    Path path;
    int index = 0;
    try {
        path  = config().blackboard->get<Path>("path");
        index = config().blackboard->get<int>("path_index");
    } catch (const std::exception& e) {
        std::cerr << "[MoveToNextWaypoint] ERROR: No path data: " << e.what() << "\n";
        return BT::NodeStatus::FAILURE;
    }

    // Check if we've reached the end
    if (index < 0 || static_cast<std::size_t>(index) >= path.size()) {
        std::cout << "[MoveToNextWaypoint] End of path reached.\n";
        return BT::NodeStatus::SUCCESS;
    }

    const Point& waypoint = path[static_cast<std::size_t>(index)];

    // Check if waypoint is traversable
    GridMap* map = nullptr;
    try { map = config().blackboard->get<GridMap*>("grid_map"); } catch (...) {}
    if (map && !map->isTraversable(waypoint)) {
        std::cerr << "[MoveToNextWaypoint] Waypoint " << waypoint << " is blocked!\n";
        return BT::NodeStatus::FAILURE;
    }

    // Advance progress
    progress_ += speed_;

    if (progress_ >= 1.0) {
        // Move complete
        config().blackboard->set("agent_x", waypoint.x);
        config().blackboard->set("agent_y", waypoint.y);
        config().blackboard->set("path_index", index + 1);

        int remaining = static_cast<int>(path.size()) - index - 1;
        std::cout << "[MoveToNextWaypoint] Moved to waypoint #" << index
                  << " " << waypoint << " (remaining: " << remaining << ")\n";
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void MoveToNextWaypoint::onHalted() {
    std::cout << "[MoveToNextWaypoint] Movement halted.\n";
    progress_ = 0.0;
}

// ============================================================
// MoveToTarget
// ============================================================
MoveToTarget::MoveToTarget(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList MoveToTarget::providedPorts() {
    return {};
}

BT::NodeStatus MoveToTarget::onStart() {
    std::cout << "[MoveToTarget] Starting direct movement toward target...\n";
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveToTarget::onRunning() {
    int cx, cy, gx, gy;
    try {
        cx = config().blackboard->get<int>("agent_x");
        cy = config().blackboard->get<int>("agent_y");
        gx = config().blackboard->get<int>("target_x");
        gy = config().blackboard->get<int>("target_y");
    } catch (const std::exception&) {
        return BT::NodeStatus::FAILURE;
    }

    // Already at target
    if (cx == gx && cy == gy) {
        std::cout << "[MoveToTarget] Already at target.\n";
        return BT::NodeStatus::SUCCESS;
    }

    // Simple gradient-descent: step toward target
    int dx = (gx > cx) ? 1 : (gx < cx) ? -1 : 0;
    int dy = (gy > cy) ? 1 : (gy < cy) ? -1 : 0;
    int nx = cx + dx;
    int ny = cy + dy;

    // Check if next cell is traversable
    GridMap* map = nullptr;
    try { map = config().blackboard->get<GridMap*>("grid_map"); } catch (...) {}

    if (map && !map->isTraversable(nx, ny)) {
        // Try alternative directions
        bool moved = false;
        for (const auto& dir : kCardinalDirs) {
            int ax2 = cx + dir[0];
            int ay2 = cy + dir[1];
            if (map->isTraversable(ax2, ay2)) {
                int newDist = std::abs(ax2 - gx) + std::abs(ay2 - gy);
                int curDist = std::abs(cx - gx) + std::abs(cy - gy);
                if (newDist < curDist) {
                    nx = ax2; ny = ay2;
                    moved = true;
                    break;
                }
            }
        }
        if (!moved) {
            std::cerr << "[MoveToTarget] Stuck - all directions blocked.\n";
            return BT::NodeStatus::FAILURE;
        }
    }

    config().blackboard->set("agent_x", nx);
    config().blackboard->set("agent_y", ny);

    std::cout << "[MoveToTarget] Simple step to (" << nx << ", " << ny << ")\n";

    if (nx == gx && ny == gy) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void MoveToTarget::onHalted() {
    std::cout << "[MoveToTarget] Direct movement halted.\n";
}

// ============================================================
// RotateTowardTarget
// ============================================================
RotateTowardTarget::RotateTowardTarget(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList RotateTowardTarget::providedPorts() {
    return {};
}

BT::NodeStatus RotateTowardTarget::onStart() {
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RotateTowardTarget::onRunning() {
    int ax, ay, tx, ty;
    try {
        ax = config().blackboard->get<int>("agent_x");
        ay = config().blackboard->get<int>("agent_y");
        tx = config().blackboard->get<int>("target_x");
        ty = config().blackboard->get<int>("target_y");
    } catch (const std::exception&) {
        return BT::NodeStatus::FAILURE;
    }

    double current = 0.0;
    try {
        current = config().blackboard->get<double>("agent_heading");
    } catch (...) {
        config().blackboard->set("agent_heading", 0.0);
    }

    // Compute desired heading
    double desired = std::atan2(
        static_cast<double>(ty - ay),
        static_cast<double>(tx - ax)
    );

    // Normalize angle difference to [-PI, PI]
    double diff = desired - current;
    while (diff >  M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;

    const double kAngleThreshold = 0.05;

    if (std::abs(diff) < kAngleThreshold) {
        config().blackboard->set("agent_heading", desired);
        std::cout << "[RotateTowardTarget] Facing target. Heading=" << desired << "\n";
        return BT::NodeStatus::SUCCESS;
    }

    // Rotate with a max rate
    const double kMaxRotation = 0.3;
    double step = std::clamp(diff, -kMaxRotation, kMaxRotation);
    double newHeading = current + step;
    while (newHeading >  M_PI) newHeading -= 2.0 * M_PI;
    while (newHeading < -M_PI) newHeading += 2.0 * M_PI;

    config().blackboard->set("agent_heading", newHeading);
    std::cout << "[RotateTowardTarget] Rotating: heading=" << newHeading
              << " (diff=" << diff << ")\n";
    return BT::NodeStatus::RUNNING;
}

void RotateTowardTarget::onHalted() {
    std::cout << "[RotateTowardTarget] Rotation halted.\n";
}

// ============================================================
// WaitAtObstacle
// ============================================================
WaitAtObstacle::WaitAtObstacle(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList WaitAtObstacle::providedPorts() {
    return {
        BT::InputPort<int>("wait_max", 5, "Maximum wait attempts")
    };
}

BT::NodeStatus WaitAtObstacle::onStart() {
    int count = 0;
    try { count = config().blackboard->get<int>("wait_count"); } catch (...) {}

    waitTicks_ = 1 << count;

    auto maxOpt = getInput<int>("wait_max");
    maxWaitTicks_ = maxOpt.value_or(5);

    if (count >= maxWaitTicks_) {
        std::cerr << "[WaitAtObstacle] Max wait attempts reached.\n";
        config().blackboard->set("wait_count", 0);
        return BT::NodeStatus::FAILURE;
    }

    config().blackboard->set("wait_count", count + 1);
    std::cout << "[WaitAtObstacle] Waiting " << waitTicks_ << " ticks "
              << "(attempt " << (count + 1) << "/" << maxWaitTicks_ << ")\n";
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitAtObstacle::onRunning() {
    if (waitTicks_ > 0) {
        --waitTicks_;
        return BT::NodeStatus::RUNNING;
    }
    std::cout << "[WaitAtObstacle] Wait complete.\n";
    return BT::NodeStatus::SUCCESS;
}

void WaitAtObstacle::onHalted() {
    std::cout << "[WaitAtObstacle] Wait interrupted.\n";
    waitTicks_ = 0;
}

// ============================================================
// RecomputePath
// ============================================================
RecomputePath::RecomputePath(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList RecomputePath::providedPorts() {
    return {};
}

BT::NodeStatus RecomputePath::tick() {
    int ax, ay, tx, ty;
    try {
        ax = config().blackboard->get<int>("agent_x");
        ay = config().blackboard->get<int>("agent_y");
        tx = config().blackboard->get<int>("target_x");
        ty = config().blackboard->get<int>("target_y");
    } catch (const std::exception& e) {
        std::cerr << "[RecomputePath] ERROR: " << e.what() << "\n";
        return BT::NodeStatus::FAILURE;
    }

    Point start(ax, ay);
    Point goal(tx, ty);

    AStarPathfinder localFinder;
    AStarPathfinder* finder = pathfinder_ ? pathfinder_ : &localFinder;

    GridMap* map = nullptr;
    try { map = config().blackboard->get<GridMap*>("grid_map"); } catch (...) {}
    if (map) finder->setMap(map);

    std::cout << "[RecomputePath] Recomputing A* path from "
              << start << " to " << goal << "...\n";

    Path path;
    bool found = finder->findPath(start, goal, path);

    if (found) {
        config().blackboard->set("path", path);
        config().blackboard->set("path_index", 0);
        config().blackboard->set("path_found", true);
        std::cout << "[RecomputePath] New path found. Length=" << path.size() << "\n";
        return BT::NodeStatus::SUCCESS;
    }

    std::cerr << "[RecomputePath] No path found.\n";
    return BT::NodeStatus::FAILURE;
}

// ============================================================
// Factory registration
// ============================================================
void registerNavigationNodes(BT::BehaviorTreeFactory& factory,
                              AStarPathfinder* pathfinder,
                              GridMap* map) {
    // Action nodes
    factory.registerNodeType<SetTarget>("SetTarget");
    factory.registerNodeType<UpdateAgentPosition>("UpdateAgentPosition");
    factory.registerNodeType<ComputePath>("ComputePath");
    factory.registerNodeType<MoveToNextWaypoint>("MoveToNextWaypoint");
    factory.registerNodeType<MoveToTarget>("MoveToTarget");
    factory.registerNodeType<RotateTowardTarget>("RotateTowardTarget");
    factory.registerNodeType<WaitAtObstacle>("WaitAtObstacle");
    factory.registerNodeType<RecomputePath>("RecomputePath");

    // Condition nodes
    factory.registerNodeType<HasTarget>("HasTarget");
    factory.registerNodeType<HasPath>("HasPath");
    factory.registerNodeType<IsTargetReached>("IsTargetReached");
    factory.registerNodeType<IsPathBlocked>("IsPathBlocked");
    factory.registerNodeType<IsObstacleNearby>("IsObstacleNearby");

    std::cout << "[RegisterNodes] Registered " << 13
              << " navigation behavior tree nodes.\n";
}

}  // namespace nav_bt
