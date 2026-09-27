#include "algorithms/BFS.hpp"
#include <queue>
#include <chrono>

namespace Pathfinding {

PathfindingResult BFSAlgorithm::findPath(
    const Grid& grid,
    Point start,
    Point target,
    StepCallback callback
) {
    PathfindingResult result;
    result.algorithmName = getName();

    auto startTime = std::chrono::high_resolution_clock::now();

    if (!grid.isValid(start) || !grid.isValid(target) ||
        !grid.isWalkable(start) || !grid.isWalkable(target)) {
        auto endTime = std::chrono::high_resolution_clock::now();
        result.durationMicroseconds = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
        return result;
    }

    if (start == target) {
        result.found = true;
        result.path = { start };
        result.totalCost = 0.0;
        result.nodesExplored = 1;
        auto endTime = std::chrono::high_resolution_clock::now();
        result.durationMicroseconds = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
        return result;
    }

    std::queue<Point> frontier;
    std::unordered_map<Point, Point> cameFrom;
    std::vector<std::vector<bool>> visited(grid.getHeight(), std::vector<bool>(grid.getWidth(), false));

    frontier.push(start);
    visited[start.y][start.x] = true;

    bool reached = false;

    while (!frontier.empty()) {
        Point current = frontier.front();
        frontier.pop();

        result.nodesExplored++;
        result.visitedOrder.push_back(current);

        if (callback && current != start && current != target) {
            callback(current, CellType::Visited);
        }

        if (current == target) {
            reached = true;
            break;
        }

        for (const Point& neighbor : grid.getNeighbors(current)) {
            if (!visited[neighbor.y][neighbor.x]) {
                visited[neighbor.y][neighbor.x] = true;
                cameFrom[neighbor] = current;
                frontier.push(neighbor);

                if (callback && neighbor != start && neighbor != target) {
                    callback(neighbor, CellType::Frontier);
                }
            }
        }
    }

    if (reached) {
        result.found = true;
        result.path = reconstructPath(cameFrom, target, start);
        result.totalCost = calculatePathCost(grid, result.path);

        if (callback) {
            for (const Point& p : result.path) {
                if (p != start && p != target) {
                    callback(p, CellType::Path);
                }
            }
        }
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    result.durationMicroseconds = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
    return result;
}

} // namespace Pathfinding
