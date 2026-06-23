/**
 * @file navigation_nodes.h
 * @brief Behavior Tree action nodes for robot/agent navigation.
 *
 * Action nodes modify the agent's state and the blackboard. They tick
 * for multiple cycles (returning RUNNING) until their task completes.
 *
 * Nodes:
 *   - ComputePath:       Runs A* to compute a path from agent to target
 *   - MoveToNextWaypoint: Moves agent one step along the computed path
 *   - MoveToTarget:      Direct movement toward target (no pre-computed path)
 *   - RotateTowardTarget: Orient the agent toward the target
 *   - WaitAtObstacle:    Pause when blocked (back-off behavior)
 *   - RecomputePath:     Clear and recompute the path (used in fallback chain)
 *   - SetTarget:         Set the navigation target on the blackboard
 *   - UpdateAgentPosition: Update agent position from sensors
 */

#ifndef NAVIGATION_BT_NAVIGATION_NODES_H_
#define NAVIGATION_BT_NAVIGATION_NODES_H_

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

#include "core/types.h"
#include "core/grid_map.h"
#include "algorithms/astar.h"

namespace nav_bt {

// ============================================================
// SetTarget
// ============================================================
/**
 * Sets the navigation target coordinates on the blackboard.
 *
 * Blackboard outputs:
 *   - target_x, target_y (int): Target position
 *   - target_set (bool): Set to true
 *
 * Ports (input):
 *   - x (int): Target X coordinate
 *   - y (int): Target Y coordinate
 */
class SetTarget : public BT::SyncActionNode {
public:
    SetTarget(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

// ============================================================
// UpdateAgentPosition
// ============================================================
/**
 * Reads the agent's current position and writes it to the blackboard.
 * In a real system this would read from odometry/GPS/sensors.
 *
 * Blackboard outputs:
 *   - agent_x, agent_y (int): Current agent position
 *
 * Ports (input):
 *   - x, y (int): Optional override values
 */
class UpdateAgentPosition : public BT::SyncActionNode {
public:
    UpdateAgentPosition(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

// ============================================================
// ComputePath
// ============================================================
/**
 * Runs the A* algorithm to compute a path from the current agent position
 * to the specified target.
 *
 * Blackboard inputs:
 *   - agent_x, agent_y (int): Start position
 *   - target_x, target_y (int): Goal position
 *   - grid_map (GridMap*): The map to search on
 *
 * Blackboard outputs:
 *   - path (std::vector<Point>): The computed path
 *   - path_index (int): Set to 0 (start of path)
 *   - path_found (bool): Whether a path was found
 *   - astar_stats (AStarStats): Search statistics
 *
 * Returns SUCCESS if path found, FAILURE if no path exists.
 */
class ComputePath : public BT::SyncActionNode {
public:
    ComputePath(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();

    /// Configure the pathfinder
    void setPathfinder(AStarPathfinder* pathfinder) { pathfinder_ = pathfinder; }

private:
    BT::NodeStatus tick() override;
    AStarPathfinder* pathfinder_ = nullptr;
};

// ============================================================
// MoveToNextWaypoint
// ============================================================
/**
 * Moves the agent one step toward the next waypoint on the path.
 *
 * Blackboard inputs:
 *   - path: The full path
 *   - path_index (int): Index of the next waypoint
 *   - agent_x, agent_y (int): Current position (updated on success)
 *   - grid_map (GridMap*): Map for validation
 *
 * Blackboard outputs:
 *   - agent_x, agent_y: Updated position
 *   - path_index: Incremented
 *
 * Returns SUCCESS when the step is complete, RUNNING while moving,
 *         FAILURE if the next waypoint is unreachable.
 */
class MoveToNextWaypoint : public BT::StatefulActionNode {
public:
    MoveToNextWaypoint(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    /// Set movement speed (cells per tick, default 1)
    void setSpeed(double speed) { speed_ = speed; }

private:
    double speed_ = 1.0;
    double progress_ = 0.0;
};

// ============================================================
// MoveToTarget
// ============================================================
/**
 * Moves the agent directly toward the target one step at a time
 * (simpler alternative to path following — used as fallback).
 *
 * Uses a simple gradient-descent style movement: move toward target
 * while checking for obstacles.
 *
 * Returns RUNNING while moving, SUCCESS when arrived, FAILURE if stuck.
 */
class MoveToTarget : public BT::StatefulActionNode {
public:
    MoveToTarget(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

// ============================================================
// RotateTowardTarget
// ============================================================
/**
 * Rotates the agent to face the target. Uses linear interpolation
 * of heading angle.
 *
 * Blackboard:
 *   - agent_heading (double): Current heading in radians
 *   - target_x, target_y, agent_x, agent_y
 *
 * Returns RUNNING while rotating, SUCCESS when aligned.
 */
class RotateTowardTarget : public BT::StatefulActionNode {
public:
    RotateTowardTarget(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

// ============================================================
// WaitAtObstacle
// ============================================================
/**
 * Pauses execution when blocked by an obstacle. Implements
 * a back-off timer with exponential backoff.
 *
 * Blackboard:
 *   - wait_count (int): Number of times we've waited (for backoff)
 *   - wait_max (int, default=5): Max wait attempts before giving up
 *
 * Returns RUNNING during wait, SUCCESS when wait completes,
 *         FAILURE when max waits exceeded.
 */
class WaitAtObstacle : public BT::StatefulActionNode {
public:
    WaitAtObstacle(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    int waitTicks_ = 0;
    int maxWaitTicks_ = 5;
};

// ============================================================
// RecomputePath
// ============================================================
/**
 * Clears the current path and triggers a new A* computation.
 * Used in a fallback chain when MoveToNextWaypoint fails.
 *
 * Returns SUCCESS if recomputation succeeded, FAILURE otherwise.
 */
class RecomputePath : public BT::SyncActionNode {
public:
    RecomputePath(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();

    void setPathfinder(AStarPathfinder* pathfinder) { pathfinder_ = pathfinder; }

private:
    BT::NodeStatus tick() override;
    AStarPathfinder* pathfinder_ = nullptr;
};

// ============================================================
// Factory registration
// ============================================================
/// Register all navigation nodes with a BehaviorTreeFactory
void registerNavigationNodes(BT::BehaviorTreeFactory& factory,
                              AStarPathfinder* pathfinder = nullptr,
                              GridMap* map = nullptr);

}  // namespace nav_bt

#endif  // NAVIGATION_BT_NAVIGATION_NODES_H_
