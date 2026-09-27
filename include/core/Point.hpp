#ifndef POINT_HPP
#define POINT_HPP

#include <iostream>
#include <functional>
#include <vector>
#include <cmath>

namespace Pathfinding {

/**
 * @brief Represents a discrete 2D coordinate on the grid.
 */
struct Point {
    int x;
    int y;

    Point() : x(0), y(0) {}
    Point(int xVal, int yVal) : x(xVal), y(yVal) {}

    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }

    bool operator!=(const Point& other) const {
        return !(*this == other);
    }

    bool operator<(const Point& other) const {
        if (x != other.x) return x < other.x;
        return y < other.y;
    }

    Point operator+(const Point& other) const {
        return Point(x + other.x, y + other.y);
    }

    Point operator-(const Point& other) const {
        return Point(x - other.x, y - other.y);
    }

    // Distance metrics
    static double manhattanDistance(const Point& a, const Point& b) {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    }

    static double euclideanDistance(const Point& a, const Point& b) {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    static double chebyshevDistance(const Point& a, const Point& b) {
        return std::max(std::abs(a.x - b.x), std::abs(a.y - b.y));
    }

    // Common directional vectors (Up, Right, Down, Left)
    static const std::vector<Point>& get4Directions() {
        static const std::vector<Point> directions = {
            Point(0, -1), // Up
            Point(1, 0),  // Right
            Point(0, 1),  // Down
            Point(-1, 0)  // Left
        };
        return directions;
    }

    // 8-way directional vectors (including diagonals)
    static const std::vector<Point>& get8Directions() {
        static const std::vector<Point> directions = {
            Point(0, -1), Point(1, -1), Point(1, 0), Point(1, 1),
            Point(0, 1), Point(-1, 1), Point(-1, 0), Point(-1, -1)
        };
        return directions;
    }
};

inline std::ostream& operator<<(std::ostream& os, const Point& p) {
    os << "(" << p.x << ", " << p.y << ")";
    return os;
}

} // namespace Pathfinding

// Hash specialization for std::unordered_map / std::unordered_set
namespace std {
    template <>
    struct hash<Pathfinding::Point> {
        size_t operator()(const Pathfinding::Point& p) const noexcept {
            // Standard boost hash combine
            size_t h1 = hash<int>{}(p.x);
            size_t h2 = hash<int>{}(p.y);
            return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
        }
    };
}

#endif // POINT_HPP
