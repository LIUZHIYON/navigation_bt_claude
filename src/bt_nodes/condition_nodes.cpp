/**
 * @file condition_nodes.cpp
 * @brief Implementation of navigation condition nodes.
 */

#include "bt_nodes/condition_nodes.h"
#include "core/grid_map.h"

#include <cmath>

namespace nav_bt {

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
    auto targetSet = config().blackboard->get<bool>("target_set");
    if (targetSet.has_value() && targetSet.value()) {
        return BT::NodeStatus::SUCCESS;
    }

    // Check coordinates
    auto tx = config().blackboard->get<int>("target_x");
    auto ty = config().blackboard->get<int>("target_y");

    if (tx.has_value() && ty.has_value()) {
        // Valid coordinates — target is set
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
    // Check if path exists on blackboard
    auto pathOpt = config().blackboard->get<std::vector<Point>>("path");
    if (!pathOpt.has_value()) {
        return BT::NodeStatus::FAILURE;
    }

    const auto& path = pathOpt.value();
    int index = 0;
    auto idxOpt = config().blackboard->get<int>("path_index");
    if (idxOpt.has_value()) {
        index = idxOpt.value();
    }

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
    auto ax = config().blackboard->get<int>("agent_x");
    auto ay = config().blackboard->get<int>("agent_y");
    auto tx = config().blackboard->get<int>("target_x");
    auto ty = config().blackboard->get<int>("target_y");
    auto threshold = config().blackboard->get<double>("reach_threshold");

    if (!ax || !ay || !tx || !ty) {
        return BT::NodeStatus::FAILURE;
    }

    double dx = static_cast<double>(ax.value() - tx.value());
    double dy = static_cast<double>(ay.value() - ty.value());
    double dist = std::sqrt(dx * dx + dy * dy);

    double thresh = threshold.value_or(1.5);

    return (dist <= thresh) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
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
    auto pathOpt = config().blackboard->get<std::vector<Point>>("path");
    auto idxOpt  = config().blackboard->get<int>("path_index");
    auto mapOpt  = config().blackboard->get<GridMap*>("grid_map");

    if (!pathOpt || !idxOpt || !mapOpt) {
        return BT::NodeStatus::FAILURE;
    }

    const auto& path = pathOpt.value();
    int index = idxOpt.value();
    GridMap* map = mapOpt.value();

    if (map == nullptr) {
        return BT::NodeStatus::FAILURE;
    }

    // Check the next waypoint
    if (index >= 0 && static_cast<std::size_t>(index) < path.size()) {
        const Point& next = path[static_cast<std::size_t>(index)];
        return map->isTraversable(next) ? BT::NodeStatus::FAILURE
                                         : BT::NodeStatus::SUCCESS;
    }

    // No more waypoints = not blocked, just done
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
    auto ax = config().blackboard->get<int>("agent_x");
    auto ay = config().blackboard->get<int>("agent_y");
    auto mapOpt = config().blackboard->get<GridMap*>("grid_map");
    auto radiusOpt = config().blackboard->get<int>("scan_radius");

    if (!ax || !ay || !mapOpt) {
        return BT::NodeStatus::FAILURE;
    }

    GridMap* map = mapOpt.value();
    if (map == nullptr) {
        return BT::NodeStatus::FAILURE;
    }

    int radius = radiusOpt.value_or(1);

    for (int dx = -radius; dx <= radius; ++dx) {
        for (int dy = -radius; dy <= radius; ++dy) {
            if (dx == 0 && dy == 0) continue;  // skip self
            int nx = ax.value() + dx;
            int ny = ay.value() + dy;
            if (!map->isTraversable(nx, ny)) {
                return BT::NodeStatus::SUCCESS;
            }
        }
    }

    return BT::NodeStatus::FAILURE;
}

}  // namespace nav_bt
