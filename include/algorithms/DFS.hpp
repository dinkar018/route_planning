#ifndef DFS_HPP
#define DFS_HPP

#include "PathfindingAlgorithm.hpp"

namespace Pathfinding {

/**
 * @brief Depth-First Search algorithm. Explores along branch paths as deeply as possible before backtracking.
 * Does NOT guarantee the shortest path.
 */
class DFSAlgorithm : public PathfindingAlgorithm {
public:
    PathfindingResult findPath(
        const Grid& grid,
        Point start,
        Point target,
        StepCallback callback = nullptr
    ) override;

    std::string getName() const override { return "Depth-First Search (DFS)"; }
    std::string getDescription() const override {
        return "Explores deep into branches before backtracking. Fast for finding any path, but does NOT guarantee shortest path.";
    }
};

} // namespace Pathfinding

#endif // DFS_HPP
