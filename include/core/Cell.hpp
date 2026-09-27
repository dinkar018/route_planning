#ifndef CELL_HPP
#define CELL_HPP

#include "Point.hpp"
#include <string>

namespace Pathfinding {

/**
 * @brief Categorizes the state and semantic role of a cell in the grid.
 */
enum class CellType {
    Empty,       // Normal walkable terrain (cost = 1.0)
    Wall,        // Impassable obstacle
    Start,       // Route origin
    Target,      // Route destination
    Visited,     // Explored during pathfinding
    Frontier,    // Currently in open queue / frontier
    Path,        // Included in the final computed route
    Swamp        // High-cost terrain (e.g. cost = 5.0) to demonstrate weighted pathfinding
};

/**
 * @brief Represents an individual unit node on the 2D grid.
 */
class Cell {
public:
    Cell();
    Cell(const Point& pos, CellType type = CellType::Empty, double weight = 1.0);

    // Getters
    const Point& getPosition() const { return m_position; }
    CellType getType() const { return m_type; }
    double getWeight() const { return m_weight; }
    bool isWalkable() const;

    // Setters
    void setPosition(const Point& pos) { m_position = pos; }
    void setType(CellType type);
    void setWeight(double weight) { m_weight = weight; }

    // Helpers
    char toChar() const;
    std::string getAnsiColor() const;

private:
    Point m_position;
    CellType m_type;
    double m_weight;
};

} // namespace Pathfinding

#endif // CELL_HPP
