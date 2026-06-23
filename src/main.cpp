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
 *
 * Groot2 real-time monitoring:
 *   The program starts a ZMQ publisher on port 1667 (default).
 *   Open Groot2, connect to tcp://localhost:1667 to see the tree executing live.
 */

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include "core/types.h"
#include "core/grid_map.h"
#include "algorithms/astar.h"
#include "bt_nodes/condition_nodes.h"
#include "bt_nodes/navigation_nodes.h"

#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <cstdlib>

using namespace nav_bt;

// ============================================================
// Helper: Print a banner
// ============================================================
void printBanner() {
    std::cout << R"(
╔══════════════════════════════════════════════════════════╗
║     Navigation Behavior Tree Demo                       ║
║     Algorithm: A* (A-Star) Pathfinding                  ║
║     Framework: BehaviorTree.CPP v4                      ║
║     Groot2 Compatible                                   ║
╚══════════════════════════════════════════════════════════╝
)" << std::endl;
}

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

// ============================================================
// Helper: Generate a sample map if no file provided
// ============================================================
void generateSampleMap(GridMap& map) {
    const int W = 25;
    const int H = 15;
    map = GridMap(W, H);

    // Place obstacles to create a maze-like environment
    auto addWall = [&](int x1, int y1, int x2, int y2) {
        for (int y = y1; y <= y2; ++y)
            for (int x = x1; x <= x2; ++x)
                map.setObstacle(x, y, true);
    };

    // Outer walls
    addWall(0, 0, W-1, 0);           // top
    addWall(0, H-1, W-1, H-1);       // bottom
    addWall(0, 0, 0, H-1);           // left
    addWall(W-1, 0, W-1, H-1);       // right

    // Internal obstacles — create a simple maze
    addWall(5, 2, 5, 5);             // vertical wall
    addWall(8, 4, 12, 4);            // horizontal wall
    addWall(15, 1, 15, 8);           // vertical wall
    addWall(3, 8, 10, 8);            // horizontal wall
    addWall(12, 6, 12, 12);          // vertical wall
    addWall(10, 10, 20, 10);         // horizontal wall
    addWall(18, 2, 18, 7);           // vertical wall
    addWall(20, 4, 23, 4);           // small horizontal

    // Add some "rough terrain" (higher traversal cost)
    map.setCellCost(7, 6, 3.0);
    map.setCellCost(7, 7, 3.0);
    map.setCellCost(7, 9, 3.0);
    map.setCellCost(16, 9, 3.0);
    map.setCellCost(17, 9, 3.0);

    // Set start (accessible)
    map.setCellType(2, 2, CellType::START);

    // Set goal (accessible)
    map.setCellType(22, 12, CellType::GOAL);

    std::cout << "[Main] Generated sample map " << W << "x" << H << "\n";
}

// ============================================================
// Main
// ============================================================
int main(int argc, char* argv[]) {
    printBanner();

    // --- Parse arguments ---
    std::string treeFile = "../trees/navigation_main.xml";
    std::string mapFile  = "../maps/sample_map.txt";

    if (argc > 1) treeFile = argv[1];
    if (argc > 2) mapFile  = argv[2];

    // --- Setup map ---
    GridMap gridMap;
    bool mapLoaded = gridMap.loadFromFile(mapFile);
    if (!mapLoaded) {
        std::cout << "[Main] Could not load map file: " << mapFile << "\n";
        std::cout << "[Main] Generating sample map instead.\n";
        generateSampleMap(gridMap);
        // Save it for future use
        gridMap.saveToFile(mapFile);
        std::cout << "[Main] Sample map saved to: " << mapFile << "\n";
    }

    std::cout << "\n[Main] Map:\n";
    gridMap.print();
    std::cout << "\n";

    // --- Setup A* pathfinder ---
    AStarPathfinder pathfinder;
    pathfinder.setMap(&gridMap);
    pathfinder.setMovementMode(MovementMode::EIGHT_DIR);
    pathfinder.setAllowCornerCutting(true);
    pathfinder.setHeuristic(heuristic::octile);

    // --- Test A* directly ---
    std::cout << "[Main] Running A* directly as a test...\n";
    Point start(2, 2);
    Point goal(22, 12);

    // Override with map values if loaded from file
    for (int y = 0; y < gridMap.height(); ++y) {
        for (int x = 0; x < gridMap.width(); ++x) {
            if (gridMap.cellType(x, y) == CellType::START) start = Point(x, y);
            if (gridMap.cellType(x, y) == CellType::GOAL)  goal  = Point(x, y);
        }
    }

    Path testPath;
    AStarStats testStats;
    bool found = pathfinder.findPath(start, goal, testPath, &testStats);
    printStats(testStats);

    if (found) {
        std::cout << "[Main] Path found! Map with path overlay:\n";
        gridMap.clearMarkers();
        gridMap.print(&testPath);
        std::cout << "\n";
    }

    // --- Setup Behavior Tree ---
    BT::BehaviorTreeFactory factory;

    // Register ALL custom nodes
    registerNavigationNodes(factory, &pathfinder, &gridMap);

    // Also register built-in nodes we need (they're auto-registered in v4 but explicit is safer)
    // These are standard nodes from BehaviorTree.CPP

    // --- Create blackboard ---
    auto blackboard = BT::Blackboard::create();

    // Set initial blackboard values
    blackboard->set("grid_map", &gridMap);
    blackboard->set("agent_x", start.x);
    blackboard->set("agent_y", start.y);
    blackboard->set("target_x", goal.x);
    blackboard->set("target_y", goal.y);
    blackboard->set("target_set", true);
    blackboard->set("agent_heading", 0.0);
    blackboard->set("wait_count", 0);

    // --- Load behavior tree from XML ---
    std::cout << "[Main] Loading behavior tree from: " << treeFile << "\n";

    BT::Tree tree;
    try {
        tree = factory.createTreeFromFile(treeFile, blackboard);
    } catch (const std::exception& e) {
        std::cerr << "[Main] ERROR loading tree: " << e.what() << "\n";
        std::cerr << "[Main] Falling back to programmatic tree creation.\n";
        // More details in the error message about Groot2 compatibility
        return 1;
    }

    // --- Setup loggers ---
    // Console logger for debugging
    BT::StdCoutLogger coutLogger(tree);

    // ZMQ publisher for Groot2 real-time monitoring
    // Groot2 connects to tcp://localhost:1667 by default
    BT::PublisherZMQ zmqPublisher(tree, 1667, 1668);

    std::cout << "\n[Main] ========================================\n";
    std::cout << "[Main] Behavior Tree Execution Starting!\n";
    std::cout << "[Main] Groot2 ZMQ Publisher: tcp://*:1667\n";
    std::cout << "[Main] Open Groot2 → Connect to monitor live.\n";
    std::cout << "[Main] ========================================\n\n";

    // --- Main execution loop ---
    const int kMaxTicks = 200;
    int tick = 0;

    auto status = BT::NodeStatus::RUNNING;
    while (status == BT::NodeStatus::RUNNING && tick < kMaxTicks) {
        std::cout << "\n--- Tick " << tick << " ---\n";

        status = tree.tickWhileRunning();

        // Print agent position
        auto ax = blackboard->get<int>("agent_x");
        auto ay = blackboard->get<int>("agent_y");
        if (ax && ay) {
            std::cout << "  Agent @ (" << ax.value() << ", " << ay.value() << ")\n";
        }

        // Print path progress
        auto pathIdx = blackboard->get<int>("path_index");
        auto pathOpt = blackboard->get<std::vector<Point>>("path");
        if (pathIdx && pathOpt) {
            std::cout << "  Path progress: " << pathIdx.value() << "/"
                      << pathOpt.value().size() << "\n";
        }

        // Check if target reached
        auto reached = blackboard->get<bool>("target_reached");

        gridMap.clearMarkers();
        // Mark visited path positions
        if (pathOpt && pathIdx) {
            const auto& path = pathOpt.value();
            int idx = pathIdx.value();
            // Mark remaining path
            for (std::size_t i = static_cast<std::size_t>(std::max(0, idx));
                 i < path.size(); ++i) {
                gridMap.setCellType(path[i], CellType::PATH);
            }
        }

        // Mark current agent position
        if (ax && ay) {
            gridMap.setCellType(ax.value(), ay.value(), CellType::VISITED);
        }

        gridMap.print();
        std::cout << std::endl;

        // Small delay for visualization
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        ++tick;
    }

    // --- Final report ---
    std::cout << "\n[Main] ========================================\n";
    std::cout << "[Main] Execution complete after " << tick << " ticks.\n";
    std::cout << "[Main] Final status: "
              << (status == BT::NodeStatus::SUCCESS ? "SUCCESS" :
                  status == BT::NodeStatus::FAILURE ? "FAILURE" : "RUNNING")
              << "\n";

    auto finalAx = blackboard->get<int>("agent_x");
    auto finalAy = blackboard->get<int>("agent_y");
    if (finalAx && finalAy) {
        std::cout << "[Main] Final agent position: ("
                  << finalAx.value() << ", " << finalAy.value() << ")\n";
        std::cout << "[Main] Target was: (" << goal.x << ", " << goal.y << ")\n";
    }

    std::cout << "[Main] ========================================\n";

    return (status == BT::NodeStatus::SUCCESS) ? 0 : 1;
}
