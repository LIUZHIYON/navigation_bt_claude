/**
 * @file navigation_nodes.cpp
 * @brief Implementation of navigation action nodes.
 */

#include "bt_nodes/navigation_nodes.h"

#include <cmath>
#include <iostream>
#include <algorithm>

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
    auto ax = config().blackboard->get<int>("agent_x");
    auto ay = config().blackboard->get<int>("agent_y");

    if (!ax.has_value() || !ay.has_value()) {
        // First initialization — use port values
        auto xOpt = getInput<int>("x");
        auto yOpt = getInput<int>("y");
        config().blackboard->set("agent_x", xOpt.value_or(0));
        config().blackboard->set("agent_y", yOpt.value_or(0));
    }
    // Otherwise leave as-is (already updated by movement nodes)

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
    auto ax = config().blackboard->get<int>("agent_x");
    auto ay = config().blackboard->get<int>("agent_y");
    auto tx = config().blackboard->get<int>("target_x");
    auto ty = config().blackboard->get<int>("target_y");
    auto mapOpt = config().blackboard->get<GridMap*>("grid_map");

    if (!ax || !ay || !tx || !ty) {
        std::cerr << "[ComputePath] ERROR: Missing agent or target position.\n";
        return BT::NodeStatus::FAILURE;
    }

    Point start(ax.value(), ay.value());
    Point goal(tx.value(), ty.value());

    // Use the provided pathfinder or create a temporary one
    AStarPathfinder localFinder;
    AStarPathfinder* finder = pathfinder_ ? pathfinder_ : &localFinder;

    if (mapOpt.has_value() && mapOpt.value()) {
        finder->setMap(mapOpt.value());
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
    auto pathOpt = config().blackboard->get<std::vector<Point>>("path");
    auto idxOpt  = config().blackboard->get<int>("path_index");
    auto mapOpt  = config().blackboard->get<GridMap*>("grid_map");

    if (!pathOpt || !idxOpt) {
        std::cerr << "[MoveToNextWaypoint] ERROR: No path data.\n";
        return BT::NodeStatus::FAILURE;
    }

    const auto& path = pathOpt.value();
    int index = idxOpt.value();

    // Check if we've reached the end
    if (index < 0 || static_cast<std::size_t>(index) >= path.size()) {
        std::cout << "[MoveToNextWaypoint] End of path reached.\n";
        return BT::NodeStatus::SUCCESS;
    }

    const Point& waypoint = path[static_cast<std::size_t>(index)];

    // Check if waypoint is traversable
    if (mapOpt.has_value() && mapOpt.value()) {
        if (!mapOpt.value()->isTraversable(waypoint)) {
            std::cerr << "[MoveToNextWaypoint] Waypoint " << waypoint
                      << " is blocked!\n";
            return BT::NodeStatus::FAILURE;
        }
    }

    // Advance progress
    progress_ += speed_;

    if (progress_ >= 1.0) {
        // Move complete — update agent position to waypoint
        config().blackboard->set("agent_x", waypoint.x);
        config().blackboard->set("agent_y", waypoint.y);
        config().blackboard->set("path_index", index + 1);

        std::cout << "[MoveToNextWaypoint] Moved to waypoint #" << index
                  << " " << waypoint << " (remaining: "
                  << (static_cast<int>(path.size()) - index - 1) << ")\n";
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
    auto ax = config().blackboard->get<int>("agent_x");
    auto ay = config().blackboard->get<int>("agent_y");
    auto tx = config().blackboard->get<int>("target_x");
    auto ty = config().blackboard->get<int>("target_y");
    auto mapOpt = config().blackboard->get<GridMap*>("grid_map");

    if (!ax || !ay || !tx || !ty) {
        return BT::NodeStatus::FAILURE;
    }

    int cx = ax.value();
    int cy = ay.value();
    int gx = tx.value();
    int gy = ty.value();

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
    if (mapOpt.has_value() && mapOpt.value()) {
        if (!mapOpt.value()->isTraversable(nx, ny)) {
            // Try alternative directions
            bool moved = false;
            for (const auto& dir : kCardinalDirs) {
                int ax2 = cx + dir[0];
                int ay2 = cy + dir[1];
                if (mapOpt.value()->isTraversable(ax2, ay2)) {
                    // Check if this direction reduces distance
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
                std::cerr << "[MoveToTarget] Stuck — all directions blocked.\n";
                return BT::NodeStatus::FAILURE;
            }
        }
    }

    config().blackboard->set("agent_x", nx);
    config().blackboard->set("agent_y", ny);

    std::cout << "[MoveToTarget] Simple step to (" << nx << ", " << ny << ")\n";

    // Check if we've reached the target
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
    auto ax = config().blackboard->get<int>("agent_x");
    auto ay = config().blackboard->get<int>("agent_y");
    auto tx = config().blackboard->get<int>("target_x");
    auto ty = config().blackboard->get<int>("target_y");
    auto heading = config().blackboard->get<double>("agent_heading");

    if (!ax || !ay || !tx || !ty) {
        return BT::NodeStatus::FAILURE;
    }

    // Compute desired heading (angle to target)
    double desired = std::atan2(
        static_cast<double>(ty.value() - ay.value()),
        static_cast<double>(tx.value() - ax.value())
    );

    double current = heading.value_or(0.0);

    // Normalize angle difference to [-PI, PI]
    double diff = desired - current;
    while (diff >  M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;

    const double kAngleThreshold = 0.05;  // radians (~3 degrees)

    if (std::abs(diff) < kAngleThreshold) {
        config().blackboard->set("agent_heading", desired);
        std::cout << "[RotateTowardTarget] Facing target. Heading=" << desired << "\n";
        return BT::NodeStatus::SUCCESS;
    }

    // Rotate with a max rate
    const double kMaxRotation = 0.3;  // radians per tick
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
    auto waitCount = config().blackboard->get<int>("wait_count");
    int count = waitCount.value_or(0);

    // Exponential backoff: 1, 2, 4, 8, 16
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
    auto ax = config().blackboard->get<int>("agent_x");
    auto ay = config().blackboard->get<int>("agent_y");
    auto tx = config().blackboard->get<int>("target_x");
    auto ty = config().blackboard->get<int>("target_y");
    auto mapOpt = config().blackboard->get<GridMap*>("grid_map");

    if (!ax || !ay || !tx || !ty) {
        std::cerr << "[RecomputePath] ERROR: Missing position data.\n";
        return BT::NodeStatus::FAILURE;
    }

    Point start(ax.value(), ay.value());
    Point goal(tx.value(), ty.value());

    AStarPathfinder localFinder;
    AStarPathfinder* finder = pathfinder_ ? pathfinder_ : &localFinder;

    if (mapOpt.has_value() && mapOpt.value()) {
        finder->setMap(mapOpt.value());
    }

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
    // Condition nodes
    factory.registerNodeType<HasTarget>("HasTarget");
    factory.registerNodeType<HasPath>("HasPath");
    factory.registerNodeType<IsTargetReached>("IsTargetReached");
    factory.registerNodeType<IsPathBlocked>("IsPathBlocked");
    factory.registerNodeType<IsObstacleNearby>("IsObstacleNearby");

    // Action nodes
    factory.registerNodeType<SetTarget>("SetTarget");
    factory.registerNodeType<UpdateAgentPosition>("UpdateAgentPosition");
    factory.registerNodeType<ComputePath>("ComputePath");
    factory.registerNodeType<MoveToNextWaypoint>("MoveToNextWaypoint");
    factory.registerNodeType<MoveToTarget>("MoveToTarget");
    factory.registerNodeType<RotateTowardTarget>("RotateTowardTarget");
    factory.registerNodeType<WaitAtObstacle>("WaitAtObstacle");
    factory.registerNodeType<RecomputePath>("RecomputePath");

    // Register shared resources as global blackboard entries
    if (pathfinder) {
        // Store pathfinder config via blackboard
    }
    if (map) {
        // Store map pointer via blackboard
    }

    std::cout << "[RegisterNodes] Registered " << 13
              << " navigation behavior tree nodes.\n";
}

}  // namespace nav_bt
