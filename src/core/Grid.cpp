#include "core/Grid.hpp"
#include <random>
#include <algorithm>

namespace Pathfinding {

Grid::Grid(int width, int height)
    : m_width(width), m_height(height),
      m_start(1, 1), m_target(width - 2, height - 2) {
    initializeGrid();
}

void Grid::initializeGrid() {
    m_cells.clear();
    m_cells.resize(m_height, std::vector<Cell>(m_width));
    for (int y = 0; y < m_height; ++y) {
        for (int x = 0; x < m_width; ++x) {
            m_cells[y][x] = Cell(Point(x, y), CellType::Empty, 1.0);
        }
    }
    // Set boundaries as walls
    for (int x = 0; x < m_width; ++x) {
        m_cells[0][x].setType(CellType::Wall);
        m_cells[m_height - 1][x].setType(CellType::Wall);
    }
    for (int y = 0; y < m_height; ++y) {
        m_cells[y][0].setType(CellType::Wall);
        m_cells[y][m_width - 1].setType(CellType::Wall);
    }

    if (isValid(m_start)) m_cells[m_start.y][m_start.x].setType(CellType::Start);
    if (isValid(m_target)) m_cells[m_target.y][m_target.x].setType(CellType::Target);
}

void Grid::resize(int width, int height) {
    m_width = width;
    m_height = height;
    m_start = Point(1, 1);
    m_target = Point(width - 2, height - 2);
    initializeGrid();
}

bool Grid::isValid(const Point& p) const {
    return p.x >= 0 && p.x < m_width && p.y >= 0 && p.y < m_height;
}

bool Grid::isWalkable(const Point& p) const {
    if (!isValid(p)) return false;
    return m_cells[p.y][p.x].isWalkable();
}

double Grid::getWeight(const Point& p) const {
    if (!isValid(p)) return 1e9;
    return m_cells[p.y][p.x].getWeight();
}

const Cell& Grid::getCell(const Point& p) const {
    return m_cells[p.y][p.x];
}

Cell& Grid::getCell(const Point& p) {
    return m_cells[p.y][p.x];
}

const Cell& Grid::getCell(int x, int y) const {
    return m_cells[y][x];
}

Cell& Grid::getCell(int x, int y) {
    return m_cells[y][x];
}

bool Grid::setStart(const Point& p) {
    if (!isValid(p) || p == m_target) return false;
    if (isValid(m_start)) {
        m_cells[m_start.y][m_start.x].setType(CellType::Empty);
    }
    m_start = p;
    m_cells[m_start.y][m_start.x].setType(CellType::Start);
    return true;
}

bool Grid::setTarget(const Point& p) {
    if (!isValid(p) || p == m_start) return false;
    if (isValid(m_target)) {
        m_cells[m_target.y][m_target.x].setType(CellType::Empty);
    }
    m_target = p;
    m_cells[m_target.y][m_target.x].setType(CellType::Target);
    return true;
}

bool Grid::setWall(const Point& p, bool isWall) {
    if (!isValid(p) || p == m_start || p == m_target) return false;
    m_cells[p.y][p.x].setType(isWall ? CellType::Wall : CellType::Empty);
    return true;
}

bool Grid::toggleWall(const Point& p) {
    if (!isValid(p) || p == m_start || p == m_target) return false;
    if (m_cells[p.y][p.x].getType() == CellType::Wall) {
        m_cells[p.y][p.x].setType(CellType::Empty);
    } else {
        m_cells[p.y][p.x].setType(CellType::Wall);
    }
    return true;
}

bool Grid::setSwamp(const Point& p, double cost) {
    if (!isValid(p) || p == m_start || p == m_target) return false;
    m_cells[p.y][p.x].setType(CellType::Swamp);
    m_cells[p.y][p.x].setWeight(cost);
    return true;
}

std::vector<Point> Grid::getNeighbors(const Point& p, bool allowDiagonal) const {
    std::vector<Point> neighbors;
    const auto& dirs = allowDiagonal ? Point::get8Directions() : Point::get4Directions();
    for (const auto& dir : dirs) {
        Point next = p + dir;
        if (isValid(next) && isWalkable(next)) {
            neighbors.push_back(next);
        }
    }
    return neighbors;
}

void Grid::resetPathfinding() {
    for (int y = 0; y < m_height; ++y) {
        for (int x = 0; x < m_width; ++x) {
            CellType t = m_cells[y][x].getType();
            if (t == CellType::Visited || t == CellType::Frontier || t == CellType::Path) {
                // If it had custom weight, restore Swamp, else Empty
                if (m_cells[y][x].getWeight() > 1.0) {
                    m_cells[y][x].setType(CellType::Swamp);
                } else {
                    m_cells[y][x].setType(CellType::Empty);
                }
            }
        }
    }
    m_cells[m_start.y][m_start.x].setType(CellType::Start);
    m_cells[m_target.y][m_target.x].setType(CellType::Target);
}

void Grid::clearAll() {
    initializeGrid();
}

void Grid::loadPresetOpen() {
    clearAll();
}

void Grid::loadPresetCorridor() {
    clearAll();
    // Build serpentine walls
    int midY1 = m_height / 3;
    int midY2 = (2 * m_height) / 3;

    for (int x = 1; x < m_width - 4; ++x) {
        setWall(Point(x, midY1), true);
    }
    for (int x = 4; x < m_width - 1; ++x) {
        setWall(Point(x, midY2), true);
    }
}

void Grid::loadPresetObstacleWall() {
    clearAll();
    int midX = m_width / 2;
    // Central barrier with a small gap near top and bottom
    for (int y = 2; y < m_height - 2; ++y) {
        if (y != m_height / 2) {
            setWall(Point(midX, y), true);
        }
    }
}

void Grid::loadPresetMaze(unsigned int seed) {
    clearAll();
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    for (int y = 1; y < m_height - 1; ++y) {
        for (int x = 1; x < m_width - 1; ++x) {
            Point p(x, y);
            if (p == m_start || p == m_target) continue;
            // Leave a buffer around start and target
            if (Point::manhattanDistance(p, m_start) <= 2 || Point::manhattanDistance(p, m_target) <= 2) {
                continue;
            }
            if (dist(rng) < 0.28) {
                setWall(p, true);
            }
        }
    }
}

void Grid::loadPresetWeightedSwamp() {
    clearAll();
    // Center start and target horizontally
    int midY = m_height / 2;
    setStart(Point(1, midY));
    setTarget(Point(m_width - 2, midY));

    // Place a vertical swamp barrier in the middle spanning from y = 2 to m_height - 2
    // Leaving only y = 1 open as a detour corridor
    int midX = m_width / 2;
    int swampWidth = 3;

    for (int y = 2; y < m_height - 1; ++y) {
        for (int dx = -swampWidth / 2; dx <= swampWidth / 2; ++dx) {
            Point p(midX + dx, y);
            if (p != m_start && p != m_target) {
                setSwamp(p, 5.0); // Cost 5 vs normal cost 1
            }
        }
    }
}

void Grid::loadPresetUnreachable() {
    clearAll();
    // Enclose target completely in walls
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) continue;
            Point p = m_target + Point(dx, dy);
            if (isValid(p)) {
                setWall(p, true);
            }
        }
    }
}

} // namespace Pathfinding
