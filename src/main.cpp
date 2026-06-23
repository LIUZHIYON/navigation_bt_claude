/**
 * @file main.cpp
 * @brief Navigation Behavior Tree Demo — A* pathfinding on a grid map.
 *
 * Usage:
 *   ./navigation_demo [tree_xml] [map_file]
 *
 * Groot2 live monitor:
 *   Run this, then Groot2 → Monitor → Connect → tcp://localhost:1667
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

using namespace nav_bt;

// ============================================================
int main(int argc, char* argv[]) {
    std::cout << "\n"
              << "==============================================================\n"
              << "  Navigation Behavior Tree Demo\n"
              << "  Algorithm: A* (A-Star) Pathfinding\n"
              << "  Framework: BehaviorTree.CPP v4\n"
              << "==============================================================\n\n";

    // --- Parse arguments ---
    std::string treeFile = "trees/navigation_main.xml";
    std::string mapFile  = "maps/sample_map.txt";
    if (argc > 1) treeFile = argv[1];
    if (argc > 2) mapFile  = argv[2];

    // --- Load map ---
    GridMap gridMap;
    if (!gridMap.loadFromFile(mapFile)) {
        std::cerr << "[Main] Cannot load map: " << mapFile << "\n";
        return 1;
    }
    std::cout << "[Main] Map " << gridMap.width() << "x" << gridMap.height()
              << " loaded from " << mapFile << "\n\n";

    // --- Find start/goal ---
    Point start(1, 1), goal(gridMap.width()-2, gridMap.height()-2);
    for (int y = 0; y < gridMap.height(); ++y)
        for (int x = 0; x < gridMap.width(); ++x) {
            if (gridMap.cellType(x,y) == CellType::START) start = Point(x,y);
            if (gridMap.cellType(x,y) == CellType::GOAL)  goal  = Point(x,y);
        }

    gridMap.print();

    // --- A* test ---
    AStarPathfinder pathfinder;
    pathfinder.setMap(&gridMap);
    pathfinder.setMovementMode(MovementMode::EIGHT_DIR);
    pathfinder.setAllowCornerCutting(true);
    pathfinder.setHeuristic(heuristic::octile);

    std::cout << "\n[A* Test] " << start << " → " << goal << "\n";
    Path testPath;
    AStarStats stats;
    if (pathfinder.findPath(start, goal, testPath, &stats)) {
        std::cout << "  Path: " << stats.pathLength << " cells, cost=" << stats.totalCost
                  << ", explored=" << stats.nodesExplored
                  << ", time=" << stats.searchTimeMs << "ms\n\n";
        gridMap.print(&testPath);
        std::cout << "\n";
    } else {
        std::cerr << "  No path found!\n";
    }

    // --- Setup behavior tree ---
    BT::BehaviorTreeFactory factory;
    registerNavigationNodes(factory, &pathfinder, &gridMap);

    auto bb = BT::Blackboard::create();
    bb->set("grid_map",  &gridMap);
    bb->set("agent_x",   start.x);
    bb->set("agent_y",   start.y);
    bb->set("target_x",  goal.x);
    bb->set("target_y",  goal.y);
    bb->set("target_set", true);
    bb->set("agent_heading", 0.0);
    bb->set("wait_count",    0);

    BT::Tree tree;
    try {
        tree = factory.createTreeFromFile(treeFile, bb);
    } catch (const std::exception& e) {
        std::cerr << "[Main] Tree load error: " << e.what() << "\n";
        return 1;
    }

    BT::StdCoutLogger logger(tree);

    std::cout << "\n[Main] Behavior Tree starting...\n"
              << "[Main] Agent start: " << start << "  Goal: " << goal << "\n\n";

    // --- Main loop ---
    const int kMaxTicks = 500;
    int tick = 0;
    auto status = BT::NodeStatus::RUNNING;

    while (status == BT::NodeStatus::RUNNING && tick < kMaxTicks) {
        status = tree.tickWhileRunning();

        int ax = 0, ay = 0, pi = 0;
        Path path;
        try { ax = bb->get<int>("agent_x"); } catch (...) {}
        try { ay = bb->get<int>("agent_y"); } catch (...) {}
        try { pi = bb->get<int>("path_index"); } catch (...) {}
        try { path = bb->get<Path>("path"); } catch (...) {}

        // Print map every 5 ticks or on status change
        if (tick % 5 == 0 || status != BT::NodeStatus::RUNNING) {
            gridMap.clearMarkers();
            for (size_t i = (size_t)std::max(0, pi); i < path.size(); ++i)
                gridMap.setCellType(path[i], CellType::PATH);
            gridMap.setCellType(ax, ay, (ax == goal.x && ay == goal.y) ? CellType::GOAL : CellType::START);
            std::cout << "--- Tick " << tick
                      << "  Agent(" << ax << "," << ay << ")"
                      << "  Path " << pi << "/" << path.size()
                      << "  Status=" << toStr(status) << " ---\n";
            gridMap.print();
            std::cout << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        ++tick;
    }

    int fx = 0, fy = 0;
    try { fx = bb->get<int>("agent_x"); } catch (...) {}
    try { fy = bb->get<int>("agent_y"); } catch (...) {}

    std::cout << "\n==============================================================\n"
              << "  Done in " << tick << " ticks.\n"
              << "  Final position: (" << fx << "," << fy << ")  Target: (" << goal.x << "," << goal.y << ")\n"
              << "  Status: " << toStr(status) << "\n"
              << "==============================================================\n";

    return (fx == goal.x && fy == goal.y) ? 0 : 1;
}
