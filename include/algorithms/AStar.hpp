#ifndef ASTAR_HPP
#define ASTAR_HPP

#include "PathfindingAlgorithm.hpp"

namespace Pathfinding {

enum class HeuristicType {
    Manhattan,
    Euclidean
};

/**
 * @brief A* Search Algorithm. Uses heuristic f(n) = g(n) + h(n) to prioritize nodes facing the target.
 * Finds the optimal path with fewer expansions than Dijkstra.
 */
class AStarAlgorithm : public PathfindingAlgorithm {
public:
    AStarAlgorithm(HeuristicType heuristic = HeuristicType::Manhattan);

    PathfindingResult findPath(
        const Grid& grid,
        Point start,
        Point target,
        StepCallback callback = nullptr
    ) override;

    std::string getName() const override;
    std::string getDescription() const override {
        return "Heuristic informed search (f = g + h). Guides exploration directly toward the target.";
    }

    void setHeuristic(HeuristicType heuristic) { m_heuristic = heuristic; }
    HeuristicType getHeuristic() const { return m_heuristic; }

private:
    HeuristicType m_heuristic;
    double calculateHeuristic(const Point& a, const Point& b) const;
};

} // namespace Pathfinding

#endif // ASTAR_HPP
