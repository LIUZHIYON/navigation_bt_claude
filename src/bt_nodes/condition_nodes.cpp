/**
 * @file condition_nodes.cpp
 * @brief Implementation of navigation condition nodes.
 */

#include "bt_nodes/condition_nodes.h"
#include "core/grid_map.h"

#include <cmath>

namespace nav_bt {

// ============================================================
// Blackboard helper: safe get that returns false on error
// ============================================================
namespace {
template <typename T>
bool safeGet(BT::Blackboard::Ptr bb, const std::string& key, T& out) {
    try {
        out = bb->get<T>(key);
        return true;
    } catch (...) {
        return false;
    }
}
}  // namespace

// ============================================================
// HasTarget
// ============================================================
HasTarget::HasTarget(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

BT::PortsList HasTarget::providedPorts() {
    return {
        BT::InputPort<int>("target_x"),
        BT::InputPort<int>("target_y"),
        BT::InputPort<bool>("target_set")
    };
}

BT::NodeStatus HasTarget::tick() {
    // Check explicit flag first
    bool targetSet = false;
    if (safeGet(config().blackboard, "target_set", targetSet) && targetSet) {
        return BT::NodeStatus::SUCCESS;
    }

    int tx = 0, ty = 0;
    bool hasX = safeGet(config().blackboard, "target_x", tx);
    bool hasY = safeGet(config().blackboard, "target_y", ty);
    if (hasX && hasY) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

// ============================================================
// HasPath
// ============================================================
HasPath::HasPath(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

BT::PortsList HasPath::providedPorts() {
    return {
        BT::InputPort<int>("path_index", 0, "Current index along the path")
    };
}

BT::NodeStatus HasPath::tick() {
    Path path;
    if (!safeGet(config().blackboard, "path", path)) {
        return BT::NodeStatus::FAILURE;
    }

    int index = 0;
    safeGet(config().blackboard, "path_index", index);

    if (index >= 0 && static_cast<std::size_t>(index) < path.size()) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

// ============================================================
// IsTargetReached
// ============================================================
IsTargetReached::IsTargetReached(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

BT::PortsList IsTargetReached::providedPorts() {
    return {
        BT::InputPort<int>("agent_x"),
        BT::InputPort<int>("agent_y"),
        BT::InputPort<int>("target_x"),
        BT::InputPort<int>("target_y"),
        BT::InputPort<double>("reach_threshold", 1.5, "Distance threshold for arrival")
    };
}

BT::NodeStatus IsTargetReached::tick() {
    int ax = 0, ay = 0, tx = 0, ty = 0;
    if (!safeGet(config().blackboard, "agent_x", ax) ||
        !safeGet(config().blackboard, "agent_y", ay) ||
        !safeGet(config().blackboard, "target_x", tx) ||
        !safeGet(config().blackboard, "target_y", ty)) {
        return BT::NodeStatus::FAILURE;
    }

    double threshold = 1.5;
    safeGet(config().blackboard, "reach_threshold", threshold);

    double dx = static_cast<double>(ax - tx);
    double dy = static_cast<double>(ay - ty);
    double dist = std::sqrt(dx * dx + dy * dy);

    return (dist <= threshold) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ============================================================
// IsPathBlocked
// ============================================================
IsPathBlocked::IsPathBlocked(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

BT::PortsList IsPathBlocked::providedPorts() {
    return {
        BT::InputPort<int>("path_index")
    };
}

BT::NodeStatus IsPathBlocked::tick() {
    Path path;
    int index = 0;
    GridMap* map = nullptr;

    if (!safeGet(config().blackboard, "path", path) ||
        !safeGet(config().blackboard, "path_index", index) ||
        !safeGet(config().blackboard, "grid_map", map) || !map) {
        return BT::NodeStatus::FAILURE;
    }

    if (index >= 0 && static_cast<std::size_t>(index) < path.size()) {
        const Point& next = path[static_cast<std::size_t>(index)];
        return map->isTraversable(next) ? BT::NodeStatus::FAILURE
                                         : BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

// ============================================================
// IsObstacleNearby
// ============================================================
IsObstacleNearby::IsObstacleNearby(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

BT::PortsList IsObstacleNearby::providedPorts() {
    return {
        BT::InputPort<int>("agent_x"),
        BT::InputPort<int>("agent_y"),
        BT::InputPort<int>("scan_radius", 1, "Cells to scan in each direction")
    };
}

BT::NodeStatus IsObstacleNearby::tick() {
    int ax = 0, ay = 0;
    GridMap* map = nullptr;

    if (!safeGet(config().blackboard, "agent_x", ax) ||
        !safeGet(config().blackboard, "agent_y", ay) ||
        !safeGet(config().blackboard, "grid_map", map) || !map) {
        return BT::NodeStatus::FAILURE;
    }

    int radius = 1;
    safeGet(config().blackboard, "scan_radius", radius);

    for (int dx = -radius; dx <= radius; ++dx) {
        for (int dy = -radius; dy <= radius; ++dy) {
            if (dx == 0 && dy == 0) continue;
            int nx = ax + dx;
            int ny = ay + dy;
            if (!map->isTraversable(nx, ny)) {
                return BT::NodeStatus::SUCCESS;
            }
        }
    }
    return BT::NodeStatus::FAILURE;
}

}  // namespace nav_bt
