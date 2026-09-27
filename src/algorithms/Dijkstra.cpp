#include "algorithms/Dijkstra.hpp"
#include <queue>
#include <chrono>
#include <limits>

namespace Pathfinding {

namespace {
    struct DijkstraNode {
        Point point;
        double cost;

        bool operator>(const DijkstraNode& other) const {
            return cost > other.cost;
        }
    };
}

PathfindingResult DijkstraAlgorithm::findPath(
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

    int H = grid.getHeight();
    int W = grid.getWidth();
    const double INF = std::numeric_limits<double>::infinity();

    std::vector<std::vector<double>> dist(H, std::vector<double>(W, INF));
    std::vector<std::vector<bool>> closed(H, std::vector<bool>(W, false));
    std::unordered_map<Point, Point> cameFrom;

    std::priority_queue<DijkstraNode, std::vector<DijkstraNode>, std::greater<DijkstraNode>> pq;

    dist[start.y][start.x] = 0.0;
    pq.push({start, 0.0});

    bool reached = false;

    while (!pq.empty()) {
        DijkstraNode current = pq.top();
        pq.pop();

        if (closed[current.point.y][current.point.x]) {
            continue;
        }
        closed[current.point.y][current.point.x] = true;

        result.nodesExplored++;
        result.visitedOrder.push_back(current.point);

        if (callback && current.point != start && current.point != target) {
            callback(current.point, CellType::Visited);
        }

        if (current.point == target) {
            reached = true;
            break;
        }

        for (const Point& neighbor : grid.getNeighbors(current.point)) {
            if (closed[neighbor.y][neighbor.x]) continue;

            double edgeCost = grid.getWeight(neighbor);
            double newDist = dist[current.point.y][current.point.x] + edgeCost;

            if (newDist < dist[neighbor.y][neighbor.x]) {
                dist[neighbor.y][neighbor.x] = newDist;
                cameFrom[neighbor] = current.point;
                pq.push({neighbor, newDist});

                if (callback && neighbor != start && neighbor != target) {
                    callback(neighbor, CellType::Frontier);
                }
            }
        }
    }

    if (reached) {
        result.found = true;
        result.path = reconstructPath(cameFrom, target, start);
        result.totalCost = dist[target.y][target.x];

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
