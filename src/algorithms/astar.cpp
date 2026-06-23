/**
 * @file astar.cpp
 * @brief Implementation of the A* pathfinding algorithm.
 *
 * A* Search Algorithm (Hart, Nilsson & Raphael, 1968):
 *
 * f(n) = g(n) + h(n)
 *   g(n) = actual cost from start to node n
 *   h(n) = heuristic estimated cost from n to goal
 *
 * Uses a binary min-heap (std::priority_queue) as the open list for O(log N)
 * operations, and an unordered_map for closed-set / parent tracking.
 *
 * The heuristic must be admissible (never overestimate) to guarantee
 * optimality. Octile distance is used by default for 8-directional grids.
 */

#include "algorithms/astar.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <iostream>

namespace nav_bt {

// ============================================================
// findPath (core implementation)
// ============================================================
bool AStarPathfinder::findPath(const Point& start, const Point& goal,
                                Path& outPath, AStarStats* outStats) {
    outPath.clear();

    auto tStart = std::chrono::steady_clock::now();

    // --- Basic validation ---
    if (!map_) {
        std::cerr << "[AStar] ERROR: No map set.\n";
        return false;
    }

    if (!map_->isInBounds(start) || !map_->isInBounds(goal)) {
        std::cerr << "[AStar] ERROR: Start or goal out of bounds. "
                  << "Start=" << start << " Goal=" << goal << "\n";
        return false;
    }

    if (!map_->isTraversable(start)) {
        std::cerr << "[AStar] ERROR: Start position is blocked: " << start << "\n";
        return false;
    }

    if (!map_->isTraversable(goal)) {
        std::cerr << "[AStar] ERROR: Goal position is blocked: " << goal << "\n";
        return false;
    }

    if (start == goal) {
        outPath.push_back(start);
        if (outStats) {
            outStats->pathFound = true;
            outStats->pathLength = 1;
            outStats->totalCost = 0.0;
            outStats->nodesExplored = 1;
            outStats->searchTimeMs = 0.0;
        }
        return true;
    }

    // --- Data structures ---
    // gCosts: best known cost from start to each node
    std::unordered_map<Point, double, Point::Hash> gCosts;

    // cameFrom: parent pointer for path reconstruction
    std::unordered_map<Point, Point, Point::Hash> cameFrom;

    // closed set for visited nodes
    std::unordered_set<Point, Point::Hash> closed;

    // Open list: min-heap ordered by fCost
    // We use a priority_queue of AStarNode
    auto compare = [](const AStarNode& a, const AStarNode& b) {
        return a < b;  // operator< already reverses for min-heap
    };
    std::priority_queue<AStarNode, std::vector<AStarNode>, decltype(compare)> open(compare);

    // --- Initialize ---
    double hStart = heuristic_(start, goal);
    gCosts[start] = 0.0;

    open.push(AStarNode{start, 0.0, hStart, Point{-1, -1}, false, true});

    int nodesExplored = 0;
    int nodesOpened = 1;

    bool useEightDir = (movementMode_ == MovementMode::EIGHT_DIR);

    // --- Main search loop ---
    while (!open.empty()) {
        AStarNode current = open.top();
        open.pop();

        // Skip if this node has been closed (lazy deletion from heap)
        if (closed.count(current.pos) > 0) {
            continue;
        }

        ++nodesExplored;
        closed.insert(current.pos);

        // --- Goal check ---
        if (current.pos == goal) {
            outPath = reconstructPath(start, goal, cameFrom);

            // Compute total cost
            double totalCost = 0.0;
            for (std::size_t i = 1; i < outPath.size(); ++i) {
                int dx = std::abs(outPath[i].x - outPath[i-1].x);
                int dy = std::abs(outPath[i].y - outPath[i-1].y);
                if (dx == 1 && dy == 1) {
                    totalCost += kDiagonalCost * map_->cellCost(outPath[i]);
                } else {
                    totalCost += map_->cellCost(outPath[i]);
                }
            }

            auto tEnd = std::chrono::steady_clock::now();
            if (outStats) {
                outStats->pathFound     = true;
                outStats->pathLength    = static_cast<int>(outPath.size());
                outStats->totalCost     = totalCost;
                outStats->nodesExplored = nodesExplored;
                outStats->nodesOpened   = nodesOpened;
                outStats->searchTimeMs  = std::chrono::duration<double, std::milli>(
                    tEnd - tStart).count();
            }

            return true;
        }

        // --- Expand neighbors ---
        const int kDirCount = useEightDir ? 8 : 4;
        const auto* dirs = useEightDir ? kAllDirs : kCardinalDirs;

        for (int d = 0; d < kDirCount; ++d) {
            Point neighbor(current.pos.x + dirs[d][0],
                          current.pos.y + dirs[d][1]);

            // Bounds and traversability check
            if (!map_->isTraversable(neighbor)) continue;
            if (closed.count(neighbor) > 0) continue;

            // Diagonal corner-cutting check
            if (useEightDir && d >= 4 && !allowCornerCutting_) {
                if (!isValidDiagonal(current.pos, neighbor)) {
                    continue;
                }
            }

            // Movement cost
            double moveCost = map_->cellCost(neighbor);
            if (useEightDir && d >= 4) {
                moveCost *= kDiagonalCost;  // diagonal = √2 * cell cost
            }

            double tentativeG = gCosts[current.pos] + moveCost;

            // Check if we found a better path to neighbor
            auto it = gCosts.find(neighbor);
            if (it != gCosts.end() && tentativeG >= it->second) {
                continue;  // Already have a better or equal path
            }

            // Update / insert neighbor
            gCosts[neighbor] = tentativeG;
            cameFrom[neighbor] = current.pos;

            double h = heuristic_(neighbor, goal);
            open.push(AStarNode{neighbor, tentativeG, h, current.pos, false, true});
            ++nodesOpened;
        }
    }

    // --- No path found ---
    auto tEnd = std::chrono::steady_clock::now();
    if (outStats) {
        outStats->pathFound     = false;
        outStats->pathLength    = 0;
        outStats->totalCost     = 0.0;
        outStats->nodesExplored = nodesExplored;
        outStats->nodesOpened   = nodesOpened;
        outStats->searchTimeMs  = std::chrono::duration<double, std::milli>(
            tEnd - tStart).count();
    }

    std::cerr << "[AStar] No path found from " << start << " to " << goal << ". "
              << "Explored " << nodesExplored << " nodes.\n";
    return false;
}

std::optional<Path> AStarPathfinder::findPath(const Point& start, const Point& goal,
                                               AStarStats* outStats) {
    Path path;
    if (findPath(start, goal, path, outStats)) {
        return path;
    }
    return std::nullopt;
}

// ============================================================
// Path reconstruction
// ============================================================
Path AStarPathfinder::reconstructPath(
        const Point& start, const Point& goal,
        const std::unordered_map<Point, Point, Point::Hash>& cameFrom) const {

    Path path;
    Point current = goal;

    // Walk backwards from goal to start
    while (!(current == start)) {
        path.push_back(current);
        auto it = cameFrom.find(current);
        if (it == cameFrom.end()) {
            // Should not happen if search succeeded
            std::cerr << "[AStar] ERROR: Broken parent chain during path reconstruction.\n";
            return {};
        }
        current = it->second;
    }
    path.push_back(start);

    // Reverse to get start -> goal order
    std::reverse(path.begin(), path.end());
    return path;
}

// ============================================================
// Diagonal corner-cutting check
// ============================================================
bool AStarPathfinder::isValidDiagonal(const Point& from, const Point& to) const {
    // For a diagonal move (dx, dy) where both |dx|=|dy|=1,
    // both adjacent cardinal cells must be traversable.
    // E.g., moving NE requires both N and E neighbors to be clear.

    int dx = to.x - from.x;
    int dy = to.y - from.y;

    // Not a diagonal move
    if (dx == 0 || dy == 0) return true;

    // Check the two cardinal neighbors
    Point adj1{from.x + dx, from.y};       // horizontal neighbor
    Point adj2{from.x,       from.y + dy}; // vertical neighbor

    return map_->isTraversable(adj1) && map_->isTraversable(adj2);
}

}  // namespace nav_bt
