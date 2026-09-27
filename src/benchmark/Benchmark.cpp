#include "benchmark/Benchmark.hpp"
#include "algorithms/BFS.hpp"
#include "algorithms/DFS.hpp"
#include "algorithms/Dijkstra.hpp"
#include "algorithms/AStar.hpp"

#include <iostream>
#include <iomanip>
#include <algorithm>

namespace Pathfinding {

Benchmark::Benchmark() {
    // Register the four core algorithms by default
    addAlgorithm(std::make_shared<BFSAlgorithm>());
    addAlgorithm(std::make_shared<DFSAlgorithm>());
    addAlgorithm(std::make_shared<DijkstraAlgorithm>());
    addAlgorithm(std::make_shared<AStarAlgorithm>(HeuristicType::Manhattan));
}

void Benchmark::addAlgorithm(std::shared_ptr<PathfindingAlgorithm> algo) {
    if (algo) {
        m_algorithms.push_back(algo);
    }
}

void Benchmark::clearAlgorithms() {
    m_algorithms.clear();
}

std::vector<PathfindingResult> Benchmark::run(const Grid& grid) {
    std::vector<PathfindingResult> results;
    results.reserve(m_algorithms.size());

    for (auto& algo : m_algorithms) {
        // Run algorithm without step callbacks for accurate timing
        PathfindingResult res = algo->findPath(grid, grid.getStart(), grid.getTarget(), nullptr);
        results.push_back(res);
    }

    return results;
}

void Benchmark::printResultsTable(const std::vector<PathfindingResult>& results) {
    if (results.empty()) {
        std::cout << "No benchmark results to display.\n";
        return;
    }

    std::cout << "\n========================================================================================================\n";
    std::cout << "                                  PATHFINDING ALGORITHM COMPARISON BENCHMARK                            \n";
    std::cout << "========================================================================================================\n";
    std::cout << std::left
              << std::setw(32) << "Algorithm"
              << std::setw(10) << "Found?"
              << std::setw(14) << "Path Length"
              << std::setw(14) << "Total Cost"
              << std::setw(16) << "Nodes Explored"
              << std::setw(16) << "Time (us)"
              << "\n";
    std::cout << "--------------------------------------------------------------------------------------------------------\n";

    double minCost = 1e9;
    size_t minExplored = 1e9;
    long long minTime = 1e9;
    std::string optimalAlgo = "";
    std::string mostEfficientAlgo = "";
    std::string fastestAlgo = "";

    for (const auto& res : results) {
        std::string foundStr = res.found ? "YES" : "NO";
        std::string lenStr = res.found ? std::to_string(res.getPathLength()) : "N/A";
        
        std::cout << std::left
                  << std::setw(32) << res.algorithmName
                  << std::setw(10) << foundStr
                  << std::setw(14) << lenStr;

        if (res.found) {
            std::cout << std::setw(14) << std::fixed << std::setprecision(1) << res.totalCost;
        } else {
            std::cout << std::setw(14) << "N/A";
        }

        std::cout << std::setw(16) << res.nodesExplored
                  << std::setw(16) << res.durationMicroseconds
                  << "\n";

        if (res.found) {
            if (res.totalCost < minCost) {
                minCost = res.totalCost;
                optimalAlgo = res.algorithmName;
            }
            if (res.nodesExplored < minExplored) {
                minExplored = res.nodesExplored;
                mostEfficientAlgo = res.algorithmName;
            }
            if (res.durationMicroseconds < minTime) {
                minTime = res.durationMicroseconds;
                fastestAlgo = res.algorithmName;
            }
        }
    }
    std::cout << "========================================================================================================\n";

    std::cout << "\n[Comparative Analysis & Insights]:\n";
    if (!optimalAlgo.empty()) {
        std::cout << " * Optimal Path Cost     : " << minCost << " (Achieved by: " << optimalAlgo << ")\n";
        std::cout << " * Fewest Nodes Explored : " << minExplored << " (Most targeted: " << mostEfficientAlgo << ")\n";
        std::cout << " * Fastest Execution     : " << minTime << " us (" << fastestAlgo << ")\n";
    } else {
        std::cout << " * No algorithm was able to reach the target (Path is completely blocked).\n";
    }

    // Additional pedagogical takeaways
    std::cout << "\n[Key Takeaways]:\n";
    std::cout << " - BFS: Explores uniformly like a flood-fill; guarantees minimum step count in unweighted grids.\n";
    std::cout << " - DFS: Explores branch-first; fast in simple open spaces, but path is often jagged and non-optimal.\n";
    std::cout << " - Dijkstra: Evaluates lowest path cost first; guarantees optimal solution in weighted graphs.\n";
    std::cout << " - A*: Uses heuristic distance to aggressively prune unnecessary search nodes toward the goal.\n\n";
}

} // namespace Pathfinding
