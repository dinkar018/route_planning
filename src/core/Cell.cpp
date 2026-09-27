#include "core/Cell.hpp"

namespace Pathfinding {

Cell::Cell()
    : m_position(0, 0), m_type(CellType::Empty), m_weight(1.0) {}

Cell::Cell(const Point& pos, CellType type, double weight)
    : m_position(pos), m_type(type), m_weight(weight) {
    if (m_type == CellType::Swamp && m_weight <= 1.0) {
        m_weight = 5.0; // Default swamp cost
    }
}

bool Cell::isWalkable() const {
    return m_type != CellType::Wall;
}

void Cell::setType(CellType type) {
    m_type = type;
    if (m_type == CellType::Swamp && m_weight <= 1.0) {
        m_weight = 5.0;
    } else if (m_type == CellType::Empty && m_weight > 1.0) {
        m_weight = 1.0;
    }
}

char Cell::toChar() const {
    switch (m_type) {
        case CellType::Start:    return 'S';
        case CellType::Target:   return 'E';
        case CellType::Wall:     return '#';
        case CellType::Path:     return '*';
        case CellType::Frontier: return '?';
        case CellType::Visited:  return '.';
        case CellType::Swamp:    return '~';
        case CellType::Empty:
        default:                 return ' ';
    }
}

std::string Cell::getAnsiColor() const {
    // ANSI escape color strings
    switch (m_type) {
        case CellType::Start:    return "\033[1;32m"; // Bold Green
        case CellType::Target:   return "\033[1;31m"; // Bold Red
        case CellType::Wall:     return "\033[1;90m"; // Dark Gray / High intensity
        case CellType::Path:     return "\033[1;33m"; // Bold Yellow
        case CellType::Frontier: return "\033[1;35m"; // Magenta
        case CellType::Visited:  return "\033[36m";   // Cyan
        case CellType::Swamp:    return "\033[33m";   // Brown/Yellowish
        case CellType::Empty:
        default:                 return "\033[0m";    // Reset
    }
}

} // namespace Pathfinding
