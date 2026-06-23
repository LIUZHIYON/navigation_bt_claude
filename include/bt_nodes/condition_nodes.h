/**
 * @file condition_nodes.h
 * @brief Behavior Tree condition nodes for navigation.
 *
 * Condition nodes check the state of the navigation blackboard
 * and return SUCCESS or FAILURE. They do not modify state.
 *
 * Nodes:
 *   - HasTarget:      Checks if a navigation target exists
 *   - HasPath:        Checks if a computed path exists and has remaining waypoints
 *   - IsTargetReached: Checks if the agent has arrived at the target
 *   - IsPathBlocked:  Checks if the next waypoint is blocked by an obstacle
 *   - IsObstacleNearby: Checks for obstacles in adjacent cells
 */

#ifndef NAVIGATION_BT_CONDITION_NODES_H_
#define NAVIGATION_BT_CONDITION_NODES_H_

#include <behaviortree_cpp/behavior_tree.h>

namespace nav_bt {

// ============================================================
// HasTarget
// ============================================================
/**
 * Checks whether a navigation target has been set on the blackboard.
 *
 * Blackboard inputs:
 *   - target_x (int): Target X coordinate
 *   - target_y (int): Target Y coordinate
 *   - target_set (bool, optional): Explicit flag
 *
 * Returns SUCCESS if target coordinates are valid, FAILURE otherwise.
 */
class HasTarget : public BT::ConditionNode {
public:
    HasTarget(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

// ============================================================
// HasPath
// ============================================================
/**
 * Checks if there is a valid path with remaining waypoints.
 *
 * Blackboard inputs:
 *   - path (std::vector<Point>): The computed path
 *   - path_index (int): Current index along the path
 *
 * Returns SUCCESS if path_index < path.size(), FAILURE otherwise.
 */
class HasPath : public BT::ConditionNode {
public:
    HasPath(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

// ============================================================
// IsTargetReached
// ============================================================
/**
 * Checks if the agent's current position matches the target position
 * within a configurable distance threshold.
 *
 * Blackboard inputs:
 *   - agent_x, agent_y (int): Agent's current position
 *   - target_x, target_y (int): Target position
 *   - reach_threshold (double, default=1.5): Distance threshold for "reached"
 *
 * Returns SUCCESS if distance <= threshold, FAILURE otherwise.
 */
class IsTargetReached : public BT::ConditionNode {
public:
    IsTargetReached(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

// ============================================================
// IsPathBlocked
// ============================================================
/**
 * Checks whether the next waypoint on the path is blocked by an obstacle.
 *
 * Blackboard inputs:
 *   - path: The computed path
 *   - path_index: Current position along path
 *   - grid_map (GridMap*): Pointer to the grid map
 *
 * Returns SUCCESS if the next cell is blocked, FAILURE if clear.
 */
class IsPathBlocked : public BT::ConditionNode {
public:
    IsPathBlocked(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

// ============================================================
// IsObstacleNearby
// ============================================================
/**
 * Scans adjacent cells for obstacles. Useful for reactive obstacle avoidance.
 *
 * Blackboard inputs:
 *   - agent_x, agent_y (int): Agent's position
 *   - grid_map (GridMap*): The grid map
 *   - scan_radius (int, default=1): How many cells to scan in each direction
 *
 * Returns SUCCESS if any obstacle is found within scan_radius, FAILURE otherwise.
 */
class IsObstacleNearby : public BT::ConditionNode {
public:
    IsObstacleNearby(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

}  // namespace nav_bt

#endif  // NAVIGATION_BT_CONDITION_NODES_H_
