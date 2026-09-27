#ifndef BFS_HPP
#define BFS_HPP

#include "PathfindingAlgorithm.hpp"

namespace Pathfinding {

/**
 * @brief Breadth-First Search algorithm. Guarantees the shortest path on unweighted grids.
 */
class BFSAlgorithm : public PathfindingAlgorithm {
public:
    PathfindingResult findPath(
        const Grid& grid,
        Point start,
        Point target,
        StepCallback callback = nullptr
    ) override;

    std::string getName() const override { return "Breadth-First Search (BFS)"; }
    std::string getDescription() const override {
        return "Explores outwards evenly in all directions. Guarantees minimum step count in unweighted graphs.";
    }
};

} // namespace Pathfinding

#endif // BFS_HPP
