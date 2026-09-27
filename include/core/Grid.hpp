#ifndef GRID_HPP
#define GRID_HPP

#include "Point.hpp"
#include "Cell.hpp"
#include <vector>
#include <string>

namespace Pathfinding {

/**
 * @brief Manages the 2D grid map, including cells, bounds, obstacles, and map presets.
 */
class Grid {
public:
    Grid(int width = 25, int height = 15);

    // Dimensions
    int getWidth() const { return m_width; }
    int getHeight() const { return m_height; }

    // Start & Target
    Point getStart() const { return m_start; }
    Point getTarget() const { return m_target; }
    bool setStart(const Point& p);
    bool setTarget(const Point& p);

    // Validation
    bool isValid(const Point& p) const;
    bool isWalkable(const Point& p) const;
    double getWeight(const Point& p) const;

    // Cell Access
    const Cell& getCell(const Point& p) const;
    Cell& getCell(const Point& p);
    const Cell& getCell(int x, int y) const;
    Cell& getCell(int x, int y);

    // Wall & Terrain Manipulation
    bool setWall(const Point& p, bool isWall);
    bool toggleWall(const Point& p);
    bool setSwamp(const Point& p, double cost = 5.0);

    // Graph Adjacency
    std::vector<Point> getNeighbors(const Point& p, bool allowDiagonal = false) const;

    // Resetting
    void resetPathfinding(); // Clears Visited, Frontier, Path states
    void clearAll();        // Resets grid to empty except Start and Target

    // Built-in Presets
    void loadPresetOpen();
    void loadPresetCorridor();
    void loadPresetObstacleWall();
    void loadPresetMaze(unsigned int seed = 42);
    void loadPresetWeightedSwamp();
    void loadPresetUnreachable();

    // Resize
    void resize(int width, int height);

private:
    int m_width;
    int m_height;
    Point m_start;
    Point m_target;
    std::vector<std::vector<Cell>> m_cells;

    void initializeGrid();
};

} // namespace Pathfinding

#endif // GRID_HPP
