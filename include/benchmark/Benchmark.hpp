#ifndef BENCHMARK_HPP
#define BENCHMARK_HPP

#include "core/Grid.hpp"
#include "algorithms/PathfindingAlgorithm.hpp"
#include <memory>
#include <vector>

namespace Pathfinding {

/**
 * @brief Runs comparative benchmarks across multiple pathfinding algorithms on identical grid configurations.
 */
class Benchmark {
public:
    Benchmark();

    void addAlgorithm(std::shared_ptr<PathfindingAlgorithm> algo);
    void clearAlgorithms();

    /**
     * @brief Executes all registered algorithms on the given grid and returns results.
     */
    std::vector<PathfindingResult> run(const Grid& grid);

    /**
     * @brief Prints a nicely formatted comparison table and analysis to stdout.
     */
    static void printResultsTable(const std::vector<PathfindingResult>& results);

private:
    std::vector<std::shared_ptr<PathfindingAlgorithm>> m_algorithms;
};

} // namespace Pathfinding

#endif // BENCHMARK_HPP
