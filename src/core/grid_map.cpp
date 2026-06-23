/**
 * @file grid_map.cpp
 * @brief Implementation of the GridMap class.
 */

#include "core/grid_map.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <stdexcept>

namespace nav_bt {

// ============================================================
// Construction
// ============================================================
GridMap::GridMap(int width, int height)
    : width_(width)
    , height_(height)
    , cells_(static_cast<std::size_t>(width * height))
{
}

// ============================================================
// File I/O
// ============================================================
bool GridMap::loadFromFile(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "[GridMap] ERROR: Cannot open file: " << filepath << "\n";
        return false;
    }

    std::string line;

    // Read dimensions: "width height"
    if (!std::getline(file, line)) {
        std::cerr << "[GridMap] ERROR: Empty file.\n";
        return false;
    }

    // Skip comment lines at top
    while (!line.empty() && (line[0] == '#' || line[0] == '/')) {
        if (!std::getline(file, line)) {
            std::cerr << "[GridMap] ERROR: No dimension line found.\n";
            return false;
        }
    }

    std::istringstream dimStream(line);
    if (!(dimStream >> width_ >> height_)) {
        std::cerr << "[GridMap] ERROR: Invalid dimensions: " << line << "\n";
        return false;
    }

    if (width_ <= 0 || height_ <= 0) {
        std::cerr << "[GridMap] ERROR: Invalid map size: "
                  << width_ << "x" << height_ << "\n";
        return false;
    }

    cells_.clear();
    cells_.reserve(static_cast<std::size_t>(width_ * height_));

    // Read map rows
    for (int y = 0; y < height_; ++y) {
        if (!std::getline(file, line)) {
            std::cerr << "[GridMap] ERROR: Expected " << height_
                      << " rows, got " << y << "\n";
            return false;
        }

        // Trim whitespace if line is longer
        for (int x = 0; x < width_; ++x) {
            GridCell cell;
            if (x < static_cast<int>(line.size())) {
                switch (line[x]) {
                    case '.':  cell.type = CellType::FREE;     break;
                    case '#':  cell.type = CellType::OBSTACLE;  cell.cost = kObstacleCost; break;
                    case 'S':  cell.type = CellType::START;    break;
                    case 'G':  cell.type = CellType::GOAL;     break;
                    case '@':  cell.type = CellType::OBSTACLE;  cell.cost = kObstacleCost; break;
                    case '~':  // Water / difficult terrain
                        cell.type = CellType::FREE;
                        cell.cost = 3.0;  // 3x traversal cost
                        break;
                    case ',':  // Rough terrain
                        cell.type = CellType::FREE;
                        cell.cost = 2.0;
                        break;
                    default:   cell.type = CellType::FREE;     break;
                }
            } else {
                // Line shorter than width — pad with obstacles
                cell.type = CellType::OBSTACLE;
                cell.cost = kObstacleCost;
            }
            cells_.push_back(cell);
        }
    }

    file.close();

    std::cout << "[GridMap] Loaded map " << width_ << "x" << height_
              << " from " << filepath << "\n";
    return true;
}

bool GridMap::saveToFile(const std::string& filepath) const {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "[GridMap] ERROR: Cannot write to: " << filepath << "\n";
        return false;
    }

    file << width_ << " " << height_ << "\n";
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            const auto& cell = cells_[index(x, y)];
            switch (cell.type) {
                case CellType::FREE:     file << '.'; break;
                case CellType::OBSTACLE:  file << '#'; break;
                case CellType::START:    file << 'S'; break;
                case CellType::GOAL:     file << 'G'; break;
                case CellType::PATH:     file << '*'; break;
                case CellType::VISITED:  file << '?'; break;
            }
        }
        file << "\n";
    }
    file.close();
    return true;
}

// ============================================================
// Mutators
// ============================================================
void GridMap::setCellType(int x, int y, CellType type) {
    if (!isInBounds(x, y)) return;
    auto& cell = cells_[index(x, y)];
    cell.type = type;
    if (type == CellType::OBSTACLE) {
        cell.cost = kObstacleCost;
    } else if (cell.cost >= kObstacleCost) {
        cell.cost = kDefaultCellCost;
    }
}

void GridMap::setCellCost(int x, int y, CellCost cost) {
    if (!isInBounds(x, y)) return;
    cells_[index(x, y)].cost = cost;
}

void GridMap::setObstacle(int x, int y, bool obstacle) {
    if (!isInBounds(x, y)) return;
    auto& cell = cells_[index(x, y)];
    if (obstacle) {
        cell.type = CellType::OBSTACLE;
        cell.cost = kObstacleCost;
    } else {
        cell.type = CellType::FREE;
        if (cell.cost >= kObstacleCost) {
            cell.cost = kDefaultCellCost;
        }
    }
}

void GridMap::clearMarkers() {
    for (auto& cell : cells_) {
        if (cell.type == CellType::PATH || cell.type == CellType::VISITED) {
            cell.type = CellType::FREE;
            cell.cost = kDefaultCellCost;
        }
    }
}

void GridMap::reset() {
    for (auto& cell : cells_) {
        cell.type = CellType::FREE;
        cell.cost = kDefaultCellCost;
    }
}

// ============================================================
// Visualization
// ============================================================
void GridMap::print(const Path* path) const {
    std::cout << toString(path);
}

std::string GridMap::toString(const Path* path) const {
    std::ostringstream oss;

    // Top border
    oss << "+" << std::string(static_cast<std::size_t>(width_), '-') << "+\n";

    for (int y = 0; y < height_; ++y) {
        oss << "|";
        for (int x = 0; x < width_; ++x) {
            Point p{x, y};
            bool onPath = false;

            if (path) {
                onPath = std::find(path->begin(), path->end(), p) != path->end();
            }

            if (onPath) {
                oss << "*";
            } else {
                const auto& cell = cells_[index(x, y)];
                switch (cell.type) {
                    case CellType::FREE:     oss << ' '; break;
                    case CellType::OBSTACLE:  oss << "\033[31m#\033[0m"; break;
                    case CellType::START:    oss << "\033[32mS\033[0m"; break;
                    case CellType::GOAL:     oss << "\033[33mG\033[0m"; break;
                    case CellType::PATH:     oss << "\033[36m*\033[0m"; break;
                    case CellType::VISITED:  oss << "\033[90mo\033[0m"; break;
                }
            }
        }
        oss << "|\n";
    }

    // Bottom border
    oss << "+" << std::string(static_cast<std::size_t>(width_), '-') << "+\n";

    return oss.str();
}

// ============================================================
// Neighbor queries
// ============================================================
std::vector<Point> GridMap::getCardinalNeighbors(const Point& p) const {
    std::vector<Point> neighbors;
    neighbors.reserve(4);

    for (const auto& dir : kCardinalDirs) {
        Point np{p.x + dir[0], p.y + dir[1]};
        if (isTraversable(np)) {
            neighbors.push_back(np);
        }
    }
    return neighbors;
}

std::vector<Point> GridMap::getAllNeighbors(const Point& p) const {
    std::vector<Point> neighbors;
    neighbors.reserve(8);

    for (const auto& dir : kAllDirs) {
        Point np{p.x + dir[0], p.y + dir[1]};
        if (isTraversable(np)) {
            neighbors.push_back(np);
        }
    }
    return neighbors;
}

}  // namespace nav_bt
