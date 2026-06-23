/**
 * @file grid_map.h
 * @brief 2D grid map for navigation and pathfinding.
 *
 * Represents the environment as a grid of cells. Supports:
 * - Loading from text files
 * - Setting/getting cell types and traversal costs
 * - Boundary checks
 * - Visualization (print to console)
 */

#ifndef NAVIGATION_BT_CORE_GRID_MAP_H_
#define NAVIGATION_BT_CORE_GRID_MAP_H_

#include "core/types.h"

#include <string>
#include <vector>
#include <cstdint>

namespace nav_bt {

class GridMap {
public:
    GridMap() = default;
    GridMap(int width, int height);

    // ============================================================
    // File I/O
    // ============================================================
    /// Load map from a text file.
    /// Format: first line = "width height", then height rows of width chars
    ///   '.' = free cell, '#' = obstacle, 'S' = start, 'G' = goal
    bool loadFromFile(const std::string& filepath);

    /// Save current map to file
    bool saveToFile(const std::string& filepath) const;

    // ============================================================
    // Accessors
    // ============================================================
    [[nodiscard]] int width()  const { return width_; }
    [[nodiscard]] int height() const { return height_; }
    [[nodiscard]] int totalCells() const { return width_ * height_; }

    [[nodiscard]] bool isInBounds(int x, int y) const {
        return x >= 0 && x < width_ && y >= 0 && y < height_;
    }

    [[nodiscard]] bool isInBounds(const Point& p) const {
        return isInBounds(p.x, p.y);
    }

    [[nodiscard]] bool isTraversable(int x, int y) const {
        return isInBounds(x, y) && cells_[index(x, y)].isTraversable();
    }

    [[nodiscard]] bool isTraversable(const Point& p) const {
        return isTraversable(p.x, p.y);
    }

    [[nodiscard]] CellType cellType(int x, int y) const {
        if (!isInBounds(x, y)) return CellType::OBSTACLE;
        return cells_[index(x, y)].type;
    }

    [[nodiscard]] CellType cellType(const Point& p) const {
        return cellType(p.x, p.y);
    }

    [[nodiscard]] CellCost cellCost(int x, int y) const {
        if (!isInBounds(x, y)) return kObstacleCost;
        return cells_[index(x, y)].cost;
    }

    [[nodiscard]] CellCost cellCost(const Point& p) const {
        return cellCost(p.x, p.y);
    }

    // ============================================================
    // Mutators
    // ============================================================
    void setCellType(int x, int y, CellType type);
    void setCellType(const Point& p, CellType type) { setCellType(p.x, p.y, type); }

    void setCellCost(int x, int y, CellCost cost);
    void setCellCost(const Point& p, CellCost cost) { setCellCost(p.x, p.y, cost); }

    void setObstacle(int x, int y, bool obstacle);
    void setObstacle(const Point& p, bool obstacle) { setObstacle(p.x, p.y, obstacle); }

    /// Clear all path/visited markers, keeping obstacles and terrain
    void clearMarkers();

    /// Reset entire map to free cells
    void reset();

    // ============================================================
    // Visualization
    // ============================================================
    /// Print the map to console. Optionally overlay a path.
    void print(const Path* path = nullptr) const;

    /// Get a string representation of the map
    [[nodiscard]] std::string toString(const Path* path = nullptr) const;

    // ============================================================
    // Neighbor queries
    // ============================================================
    /// Get traversable neighbor positions (4-directional)
    [[nodiscard]] std::vector<Point> getCardinalNeighbors(const Point& p) const;

    /// Get traversable neighbor positions (8-directional)
    [[nodiscard]] std::vector<Point> getAllNeighbors(const Point& p) const;

private:
    [[nodiscard]] int index(int x, int y) const { return y * width_ + x; }

    int width_  = 0;
    int height_ = 0;
    std::vector<GridCell> cells_;
};

}  // namespace nav_bt

#endif  // NAVIGATION_BT_CORE_GRID_MAP_H_
