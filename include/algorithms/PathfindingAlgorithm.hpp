#ifndef PATHFINDING_ALGORITHM_HPP
#define PATHFINDING_ALGORITHM_HPP

#include "core/Grid.hpp"
#include "PathfindingResult.hpp"
#include <functional>
#include <unordered_map>
#include <string>
#include <vector>

namespace Pathfinding {

/**
 * @brief Callback signature for step-by-step visualization or animation.
 * Parameters:
 *   point: Current point being processed/visited
 *   type:  The visual state of the cell (Visited, Frontier, Path)
 */
using StepCallback = std::function<void(const Point& point, CellType type)>;

/**
 * @brief Abstract Base Class for all pathfinding strategies (Strategy Pattern).
 */
class PathfindingAlgorithm {
public:
    virtual ~PathfindingAlgorithm() = default;

    /**
     * @brief Executes the pathfinding algorithm on the provided grid.
     * @param grid Grid environment
     * @param start Origin point
     * @param target Destination point
     * @param callback Optional visualization callback triggered on search steps
     * @return PathfindingResult containing path, metrics, and visited nodes
     */
    virtual PathfindingResult findPath(
        const Grid& grid,
        Point start,
        Point target,
        StepCallback callback = nullptr
    ) = 0;

    virtual std::string getName() const = 0;
    virtual std::string getDescription() const = 0;

protected:
    /**
     * @brief Helper to reconstruct the path backwards from target to start using parent pointers.
     */
    static std::vector<Point> reconstructPath(
        const std::unordered_map<Point, Point>& cameFrom,
        Point current,
        Point start
    ) {
        std::vector<Point> path;
        path.push_back(current);

        while (current != start) {
            auto it = cameFrom.find(current);
            if (it == cameFrom.end()) {
                break;
            }
            current = it->second;
            path.push_back(current);
        }

        std::reverse(path.begin(), path.end());
        return path;
    }

    /**
     * @brief Calculates total movement cost along a reconstructed path.
     */
    static double calculatePathCost(const Grid& grid, const std::vector<Point>& path) {
        if (path.size() <= 1) return 0.0;
        double cost = 0.0;
        // Cost is incurred when moving INTO a cell (excluding start)
        for (size_t i = 1; i < path.size(); ++i) {
            cost += grid.getWeight(path[i]);
        }
        return cost;
    }
};

} // namespace Pathfinding

#endif // PATHFINDING_ALGORITHM_HPP
