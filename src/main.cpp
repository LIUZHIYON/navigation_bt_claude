/**
 * @file main.cpp
 * @brief Standalone demonstration of the Navigation Behavior Tree system.
 *
 * This demo:
 *   1. Loads a grid map from a file (or generates one)
 *   2. Sets up the A* pathfinder
 *   3. Registers all custom behavior tree nodes
 *   4. Loads and executes the navigation behavior tree from an XML file
 *   5. Prints the map and path at each tick
 *
 * Usage:
 *   ./navigation_demo [tree_xml] [map_file]
 *
 *   Default tree:  ../trees/navigation_main.xml
 *   Default map:   ../maps/sample_map.txt
 */

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include "core/types.h"
#include "core/grid_map.h"
#include "algorithms/astar.h"
#include "bt_nodes/navigation_nodes.h"

#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <stdexcept>

using namespace nav_bt;

// ============================================================
// Helper: Print search statistics
// ============================================================
void printStats(const AStarStats& stats) {
    std::cout << "\n=== A* Search Statistics ===\n";
    std::cout << "  Path found:      " << (stats.pathFound ? "YES" : "NO") << "\n";
    std::cout << "  Path length:     " << stats.pathLength << " cells\n";
    std::cout << "  Total cost:      " << stats.totalCost << "\n";
    std::cout << "  Nodes explored:  " << stats.nodesExplored << "\n";
    std::cout << "  Nodes opened:    " << stats.nodesOpened << "\n";
    std::cout << "  Search time:     " << stats.searchTimeMs << " ms\n";
    std::cout << "============================\n\n";
}

/// Safe blackboard get helper
template <typename T>
bool safeGet(const BT::Blackboard::Ptr& bb, const std::string& key, T& out) {
    try { out = bb->get<T>(key); return true; }
    catch (...) { return false; }
}

// ============================================================
// Main
// ============================================================
int main(int argc, char* argv[]) {
    std::cout << R"(
==============================================================
  Navigation Behavior Tree Demo
  Algorithm: A* (A-Star) Pathfinding
  Framework: BehaviorTree.CPP v4
  Groot2 Compatible
==============================================================
)" << std::endl;

    // --- Parse arguments ---
    std::string treeFile = "trees/navigation_main.xml";
    std::string mapFile  = "maps/sample_map.txt";

    if (argc > 1) treeFile = argv[1];
    if (argc > 2) mapFile  = argv[2];

    // --- Load map ---
    GridMap gridMap;
    if (!gridMap.loadFromFile(mapFile)) {
        std::cerr << "[Main] ERROR: Cannot load map: " << mapFile << "\n";
        return 1;
    }

    std::cout << "\n[Main] Map loaded (" << gridMap.width()
              << "x" << gridMap.height() << "):\n";
    gridMap.print();
    std::cout << "\n";

    // --- Find start/goal positions ---
    Point start(1, 1), goal(gridMap.width()-2, gridMap.height()-2);
    bool foundStart = false, foundGoal = false;
    for (int y = 0; y < gridMap.height(); ++y) {
        for (int x = 0; x < gridMap.width(); ++x) {
            if (gridMap.cellType(x, y) == CellType::START) {
                start = Point(x, y); foundStart = true;
            }
            if (gridMap.cellType(x, y) == CellType::GOAL) {
                goal = Point(x, y); foundGoal = true;
            }
        }
    }

    // --- Setup A* pathfinder ---
    AStarPathfinder pathfinder;
    pathfinder.setMap(&gridMap);
    pathfinder.setMovementMode(MovementMode::EIGHT_DIR);
    pathfinder.setAllowCornerCutting(true);
    pathfinder.setHeuristic(heuristic::octile);

    // --- Test A* directly ---
    std::cout << "[Main] Running A* directly from " << start << " to " << goal << "...\n";
    Path testPath;
    AStarStats testStats;
    if (pathfinder.findPath(start, goal, testPath, &testStats)) {
        printStats(testStats);
        std::cout << "[Main] Path found! Map overlay:\n";
        gridMap.print(&testPath);
        std::cout << "\n";
    } else {
        std::cerr << "[Main] A* could not find a path!\n";
    }

    // --- Setup Behavior Tree ---
    BT::BehaviorTreeFactory factory;
    registerNavigationNodes(factory, &pathfinder, &gridMap);

    auto blackboard = BT::Blackboard::create();
    blackboard->set("grid_map", &gridMap);
    blackboard->set("agent_x", start.x);
    blackboard->set("agent_y", start.y);
    blackboard->set("target_x", goal.x);
    blackboard->set("target_y", goal.y);
    blackboard->set("target_set", true);
    blackboard->set("agent_heading", 0.0);
    blackboard->set("wait_count", 0);

    // --- Load behavior tree ---
    std::cout << "[Main] Loading behavior tree: " << treeFile << "\n";
    BT::Tree tree;
    try {
        tree = factory.createTreeFromFile(treeFile, blackboard);
    } catch (const std::exception& e) {
        std::cerr << "[Main] ERROR: " << e.what() << "\n";
        return 1;
    }

    // Console logger
    BT::StdCoutLogger coutLogger(tree);

    std::cout << "\n[Main] ========================================" << std::endl;
    std::cout << "[Main] Behavior Tree Execution Starting!" << std::endl;
    std::cout << "[Main] ========================================\n" << std::endl;

    // --- Execution loop ---
    const int kMaxTicks = 200;
    int tick = 0;
    auto status = BT::NodeStatus::RUNNING;

    while (status == BT::NodeStatus::RUNNING && tick < kMaxTicks) {
        status = tree.tickWhileRunning();

        // Show agent position
        int ax = 0, ay = 0;
        safeGet(blackboard, "agent_x", ax);
        safeGet(blackboard, "agent_y", ay);
        std::cout << "[Tick " << tick << "] Agent @ (" << ax << ", " << ay << ")\n";

        // Show path progress
        int pathIdx = 0;
        Path path;
        safeGet(blackboard, "path_index", pathIdx);
        safeGet(blackboard, "path", path);
        std::cout << "  Path: " << pathIdx << "/" << path.size() << "\n";

        // Map visualization
        gridMap.clearMarkers();
        if (!path.empty()) {
            for (std::size_t i = static_cast<std::size_t>(std::max(0, pathIdx));
                 i < path.size(); ++i) {
                gridMap.setCellType(path[i], CellType::PATH);
            }
        }
        gridMap.setCellType(ax, ay, CellType::VISITED);
        gridMap.print();
        std::cout << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        ++tick;
    }

    // --- Final report ---
    int finalX = 0, finalY = 0;
    safeGet(blackboard, "agent_x", finalX);
    safeGet(blackboard, "agent_y", finalY);

    std::cout << "\n[Main] ========================================" << std::endl;
    std::cout << "[Main] Done in " << tick << " ticks." << std::endl;
    std::cout << "[Main] Final status: "
              << (status == BT::NodeStatus::SUCCESS ? "SUCCESS" :
                  status == BT::NodeStatus::FAILURE ? "FAILURE" : "RUNNING")
              << std::endl;
    std::cout << "[Main] Final position: (" << finalX << ", " << finalY << ")" << std::endl;
    std::cout << "[Main] Target was:     (" << goal.x << ", " << goal.y << ")" << std::endl;
    std::cout << "[Main] ========================================" << std::endl;

    return (status == BT::NodeStatus::SUCCESS) ? 0 : 1;
}
