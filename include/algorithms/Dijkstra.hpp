#ifndef DIJKSTRA_HPP
#define DIJKSTRA_HPP

#include "PathfindingAlgorithm.hpp"

namespace Pathfinding {

/**
 * @brief Dijkstra's Algorithm (Uniform Cost Search).
 * Guarantees optimal/lowest-cost path on weighted grids using a min-priority queue.
 */
class DijkstraAlgorithm : public PathfindingAlgorithm {
public:
    PathfindingResult findPath(
        const Grid& grid,
        Point start,
        Point target,
        StepCallback callback = nullptr
    ) override;

    std::string getName() const override { return "Dijkstra's Algorithm"; }
    std::string getDescription() const override {
        return "Priority queue search ordered by path cost from start. Guarantees the optimal path in weighted graphs.";
    }
};

} // namespace Pathfinding

#endif // DIJKSTRA_HPP
