/**
 * @file astar.h
 * @brief A* (A-Star) pathfinding algorithm implementation.
 *
 * The A* algorithm is a best-first search algorithm that finds the shortest path
 * between a start and goal node in a weighted graph. It uses a heuristic function
 * (Manhattan or Euclidean distance) to guide the search, making it more efficient
 * than Dijkstra's algorithm while still guaranteeing optimal paths when the
 * heuristic is admissible (never overestimates).
 *
 * Algorithm (from Hart, Nilsson, & Raphael, 1968):
 *   1. Initialize open list with start node
 *   2. While open list is not empty:
 *      a. Pop node with lowest f(n) = g(n) + h(n)
 *      b. If node is goal, reconstruct and return path
 *      c. Mark node as closed
 *      d. For each traversable neighbor:
 *         - Compute tentative g = current.g + movement cost
 *         - If tentative g < neighbor.g, update parent and costs
 *         - Add/update neighbor in open list
 *   3. If open list empty, no path exists
 *
 * Complexity: O(E log V) with a binary heap, where E = edges, V = vertices.
 */

#ifndef NAVIGATION_BT_ALGORITHMS_ASTAR_H_
#define NAVIGATION_BT_ALGORITHMS_ASTAR_H_

#include "core/types.h"
#include "core/grid_map.h"

#include <functional>
#include <queue>
#include <unordered_map>
#include <optional>

namespace nav_bt {

/// Heuristic function type: computes estimated cost from a given point to the goal
using HeuristicFunc = std::function<double(const Point&, const Point&)>;

// ============================================================
// Predefined heuristic functions
// ============================================================
namespace heuristic {

/// Manhattan distance: |dx| + |dy|
/// Admissible for 4-directional (cardinal-only) movement
inline double manhattan(const Point& a, const Point& b) {
    return static_cast<double>(a.manhattan(b));
}

/// Euclidean distance: sqrt(dx^2 + dy^2)
/// Admissible for 8-directional movement
inline double euclidean(const Point& a, const Point& b) {
    int dx = a.x - b.x;
    int dy = a.y - b.y;
    return std::sqrt(static_cast<double>(dx * dx + dy * dy));
}

/// Octile distance: max(|dx|,|dy|) + (√2-1)*min(|dx|,|dy|)
/// Optimal for 8-directional grid movement with diagonal cost √2
inline double octile(const Point& a, const Point& b) {
    int dx = std::abs(a.x - b.x);
    int dy = std::abs(a.y - b.y);
    if (dx < dy) std::swap(dx, dy);
    return static_cast<double>(dx) + (kDiagonalCost - 1.0) * static_cast<double>(dy);
}

/// Chebyshev distance: max(|dx|, |dy|)
/// Admissible for 8-directional movement with unit diagonal cost
inline double chebyshev(const Point& a, const Point& b) {
    return static_cast<double>(std::max(std::abs(a.x - b.x), std::abs(a.y - b.y)));
}

/// Diagonal distance (simplified octile)
inline double diagonal(const Point& a, const Point& b) {
    int dx = std::abs(a.x - b.x);
    int dy = std::abs(a.y - b.y);
    return static_cast<double>(dx + dy) + (kDiagonalCost - 2.0) * static_cast<double>(std::min(dx, dy));
}

}  // namespace heuristic

// ============================================================
// Movement mode
// ============================================================
enum class MovementMode {
    CARDINAL,     // 4-directional (up/down/left/right)
    EIGHT_DIR     // 8-directional (includes diagonals)
};

// ============================================================
// A* search statistics
// ============================================================
struct AStarStats {
    int     nodesExplored  = 0;
    int     nodesOpened    = 0;
    double  totalCost      = 0.0;
    int     pathLength     = 0;
    bool    pathFound      = false;
    double  searchTimeMs   = 0.0;
};

// ============================================================
// A* Pathfinder
// ============================================================
class AStarPathfinder {
public:
    AStarPathfinder() = default;

    /// Set the grid map to search on
    void setMap(const GridMap* map) { map_ = map; }

    /// Set the heuristic function (default: octile for 8-directional)
    void setHeuristic(HeuristicFunc h) { heuristic_ = std::move(h); }

    /// Set movement mode (default: 8-directional)
    void setMovementMode(MovementMode mode) { movementMode_ = mode; }

    /// Set whether to allow diagonal movement cutting corners through obstacles
    void setAllowCornerCutting(bool allow) { allowCornerCutting_ = allow; }

    // ============================================================
    // Main search method
    // ============================================================
    /**
     * Find the shortest path from start to goal using A*.
     *
     * @param start  Starting grid position
     * @param goal   Target grid position
     * @param outPath  [out] The computed path (includes start, excludes goal if unreachable)
     * @param outStats [out] Optional search statistics
     * @return true if a path was found, false otherwise
     */
    bool findPath(const Point& start, const Point& goal,
                  Path& outPath,
                  AStarStats* outStats = nullptr);

    /// Convenience overload returning std::optional<Path>
    std::optional<Path> findPath(const Point& start, const Point& goal,
                                 AStarStats* outStats = nullptr);

    // ============================================================
    // Configuration
    // ============================================================
    [[nodiscard]] const GridMap*     map()              const { return map_; }
    [[nodiscard]] MovementMode       movementMode()      const { return movementMode_; }
    [[nodiscard]] bool               allowCornerCutting() const { return allowCornerCutting_; }

private:
    /// Reconstruct the path by following parent pointers from goal to start
    [[nodiscard]] Path reconstructPath(
        const Point& start, const Point& goal,
        const std::unordered_map<Point, Point, Point::Hash>& cameFrom) const;

    /// Check if moving diagonally would cut a corner (both adjacent cardinals must be free)
    [[nodiscard]] bool isValidDiagonal(const Point& from, const Point& to) const;

    const GridMap* map_                = nullptr;
    HeuristicFunc  heuristic_          = heuristic::octile;
    MovementMode   movementMode_        = MovementMode::EIGHT_DIR;
    bool           allowCornerCutting_  = false;
};

}  // namespace nav_bt

#endif  // NAVIGATION_BT_ALGORITHMS_ASTAR_H_
