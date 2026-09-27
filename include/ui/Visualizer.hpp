#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include "core/Grid.hpp"
#include "algorithms/PathfindingResult.hpp"
#include <string>

namespace Pathfinding {

/**
 * @brief Renders the grid, paths, animations, and search telemetry to the console with ANSI colors.
 */
class Visualizer {
public:
    static void initConsole();
    static void clearScreen();
    static void sleepMs(int milliseconds);

    static void renderGrid(const Grid& grid);
    static void renderResult(const Grid& grid, const PathfindingResult& result);
    static void renderLegend();
};

} // namespace Pathfinding

#endif // VISUALIZER_HPP
