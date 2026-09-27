#ifndef PATHFINDING_RESULT_HPP
#define PATHFINDING_RESULT_HPP

#include "core/Point.hpp"
#include <vector>
#include <string>

namespace Pathfinding {

/**
 * @brief Encapsulates the complete outcome and performance telemetry of a pathfinding run.
 */
struct PathfindingResult {
    std::string algorithmName;
    bool found;
    std::vector<Point> path;
    std::vector<Point> visitedOrder;
    double totalCost;
    size_t nodesExplored;
    long long durationMicroseconds;

    PathfindingResult()
        : algorithmName(""), found(false), totalCost(0.0),
          nodesExplored(0), durationMicroseconds(0) {}

    size_t getPathLength() const {
        return path.empty() ? 0 : path.size() - 1; // Number of movements
    }
};

} // namespace Pathfinding

#endif // PATHFINDING_RESULT_HPP
