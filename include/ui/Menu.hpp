#ifndef MENU_HPP
#define MENU_HPP

#include "core/Grid.hpp"
#include "benchmark/Benchmark.hpp"
#include <memory>

namespace Pathfinding {

/**
 * @brief Interactive CLI menu interface controlling simulation, visualization, and algorithm execution.
 */
class Menu {
public:
    Menu();
    void run();

private:
    Grid m_grid;
    Benchmark m_benchmark;
    int m_animationDelayMs;

    void displayMainMenu();
    void handleRunAlgorithm();
    void handleBenchmark();
    void handleSelectPreset();
    void handleEditGrid();
    void handleRunTestSuite();

    void executeAlgorithmWithAnimation(std::shared_ptr<PathfindingAlgorithm> algo, bool animate);
};

} // namespace Pathfinding

#endif // MENU_HPP
