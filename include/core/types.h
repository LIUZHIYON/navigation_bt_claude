/**
 * @file types.h
 * @brief Common type definitions for the Navigation Behavior Tree project.
 *
 * Defines core data structures used across the entire project:
 * grid maps, A* pathfinding, and behavior tree nodes.
 */

#ifndef NAVIGATION_BT_CORE_TYPES_H_
#define NAVIGATION_BT_CORE_TYPES_H_

#include <cstdint>
#include <vector>
#include <ostream>
#include <limits>

namespace nav_bt {

// ============================================================
// 2D Integer Point (grid coordinates)
// ============================================================
struct Point {
    int x = 0;
    int y = 0;

    Point() = default;
    Point(int x_, int y_) : x(x_), y(y_) {}

    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
    bool operator!=(const Point& other) const {
        return !(*this == other);
    }

    /// Manhattan distance to another point
    [[nodiscard]] int manhattan(const Point& other) const {
        return std::abs(x - other.x) + std::abs(y - other.y);
    }

    /// Euclidean distance squared to another point
    [[nodiscard]] double distanceSquared(const Point& other) const {
        int dx = x - other.x;
        int dy = y - other.y;
        return static_cast<double>(dx * dx + dy * dy);
    }

    /// Hash functor for use in unordered containers
    struct Hash {
        std::size_t operator()(const Point& p) const {
            return static_cast<std::size_t>(p.x) * 73856093u ^
                   static_cast<std::size_t>(p.y) * 19349663u;
        }
    };
};

inline std::ostream& operator<<(std::ostream& os, const Point& p) {
    os << "(" << p.x << ", " << p.y << ")";
    return os;
}

// ============================================================
// Grid cell types
// ============================================================
enum class CellType : uint8_t {
    FREE      = 0,   // Traversable
    OBSTACLE  = 1,   // Blocked / wall
    START     = 2,   // Starting position
    GOAL      = 3,   // Goal position
    PATH      = 4,   // Part of the computed path
    VISITED   = 5    // Visited during search (for debug visualization)
};

/// Cost of traversing a cell (1.0 = normal, >1.0 = difficult terrain)
using CellCost = double;

constexpr CellCost kDefaultCellCost = 1.0;
constexpr CellCost kObstacleCost    = std::numeric_limits<double>::infinity();

// ============================================================
// Grid cell
// ============================================================
struct GridCell {
    CellType  type = CellType::FREE;
    CellCost  cost = kDefaultCellCost;  // Traversal cost multiplier

    [[nodiscard]] bool isTraversable() const {
        return type != CellType::OBSTACLE;
    }
};

// ============================================================
// A* search node
// ============================================================
struct AStarNode {
    Point    pos;          // Grid position
    double   gCost = 0.0;  // Cost from start to this node
    double   hCost = 0.0;  // Heuristic cost to goal
    Point    parent{-1, -1}; // Parent position for path reconstruction
    bool     closed = false;
    bool     opened = false;

    [[nodiscard]] double fCost() const { return gCost + hCost; }

    /// Priority queue comparator (min-heap by fCost, then hCost tiebreak)
    bool operator<(const AStarNode& other) const {
        if (std::abs(fCost() - other.fCost()) > 1e-9) {
            return fCost() > other.fCost();  // reverse for min-heap
        }
        return hCost > other.hCost;
    }
};

// ============================================================
// Path
// ============================================================
using Path = std::vector<Point>;

// ============================================================
// Movement directions
// ============================================================
enum class Direction {
    NORTH = 0,
    SOUTH,
    EAST,
    WEST,
    NORTHEAST,
    NORTHWEST,
    SOUTHEAST,
    SOUTHWEST
};

/// Get the Point offset for a given direction
inline Point directionOffset(Direction dir) {
    switch (dir) {
        case Direction::NORTH:     return { 0, -1};
        case Direction::SOUTH:     return { 0,  1};
        case Direction::EAST:      return { 1,  0};
        case Direction::WEST:      return {-1,  0};
        case Direction::NORTHEAST: return { 1, -1};
        case Direction::NORTHWEST: return {-1, -1};
        case Direction::SOUTHEAST: return { 1,  1};
        case Direction::SOUTHWEST: return {-1,  1};
    }
    return {0, 0};
}

/// Cardinal (4-direction) movement offsets
constexpr int kCardinalDirs[4][2] = {
    { 0, -1}, { 0, 1}, {-1, 0}, { 1, 0}
};

/// 8-direction movement offsets (cardinal + diagonal)
constexpr int kAllDirs[8][2] = {
    { 0, -1}, { 0, 1}, {-1, 0}, { 1, 0},   // cardinal
    {-1, -1}, { 1, -1}, {-1, 1}, { 1, 1}    // diagonal
};

/// Diagonal movement cost multiplier (√2 ≈ 1.414)
constexpr double kDiagonalCost = 1.4142135623730951;

}  // namespace nav_bt

#endif  // NAVIGATION_BT_CORE_TYPES_H_
