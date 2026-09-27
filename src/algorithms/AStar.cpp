#include "algorithms/AStar.hpp"
#include <queue>
#include <chrono>
#include <limits>

namespace Pathfinding {

namespace {
    struct AStarNode {
        Point point;
        double fScore; // gScore + hScore
        double gScore;

        bool operator>(const AStarNode& other) const {
            if (fScore != other.fScore) return fScore > other.fScore;
            return gScore < other.gScore; // Break ties with higher gScore (closer to target)
        }
    };
}

AStarAlgorithm::AStarAlgorithm(HeuristicType heuristic)
    : m_heuristic(heuristic) {}

std::string AStarAlgorithm::getName() const {
    if (m_heuristic == HeuristicType::Manhattan) {
        return "A* Search (Manhattan)";
    } else {
        return "A* Search (Euclidean)";
    }
}

double AStarAlgorithm::calculateHeuristic(const Point& a, const Point& b) const {
    if (m_heuristic == HeuristicType::Manhattan) {
        return Point::manhattanDistance(a, b);
    } else {
        return Point::euclideanDistance(a, b);
    }
}

PathfindingResult AStarAlgorithm::findPath(
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

    std::vector<std::vector<double>> gScore(H, std::vector<double>(W, INF));
    std::vector<std::vector<bool>> closed(H, std::vector<bool>(W, false));
    std::unordered_map<Point, Point> cameFrom;

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> openSet;

    gScore[start.y][start.x] = 0.0;
    double startH = calculateHeuristic(start, target);
    openSet.push({start, startH, 0.0});

    bool reached = false;

    while (!openSet.empty()) {
        AStarNode current = openSet.top();
        openSet.pop();

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
            double tentativeG = gScore[current.point.y][current.point.x] + edgeCost;

            if (tentativeG < gScore[neighbor.y][neighbor.x]) {
                cameFrom[neighbor] = current.point;
                gScore[neighbor.y][neighbor.x] = tentativeG;
                double f = tentativeG + calculateHeuristic(neighbor, target);
                openSet.push({neighbor, f, tentativeG});

                if (callback && neighbor != start && neighbor != target) {
                    callback(neighbor, CellType::Frontier);
                }
            }
        }
    }

    if (reached) {
        result.found = true;
        result.path = reconstructPath(cameFrom, target, start);
        result.totalCost = gScore[target.y][target.x];

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
