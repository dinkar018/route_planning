#include "ui/Menu.hpp"
#include "ui/Visualizer.hpp"
#include "benchmark/Benchmark.hpp"
#include "algorithms/BFS.hpp"
#include "algorithms/DFS.hpp"
#include "algorithms/Dijkstra.hpp"
#include "algorithms/AStar.hpp"

#include <iostream>
#include <string>

using namespace Pathfinding;

int main(int argc, char* argv[]) {
    Visualizer::initConsole();

    // Check command line flags
    if (argc > 1) {
        std::string arg = argv[1];
        if (arg == "--test") {
            std::cout << "Running Pathfinding Simulator Automated Test Suite...\n";
            Menu menu;
            // Run test suite
            Grid testGrid(20, 10);
            testGrid.loadPresetOpen();
            Point start(1, 1);
            Point target(18, 8);
            testGrid.setStart(start);
            testGrid.setTarget(target);

            BFSAlgorithm bfs;
            DFSAlgorithm dfs;
            DijkstraAlgorithm dijkstra;
            AStarAlgorithm astar;

            auto rBfs = bfs.findPath(testGrid, start, target);
            auto rDijk = dijkstra.findPath(testGrid, start, target);
            auto rAstar = astar.findPath(testGrid, start, target);

            bool pass1 = (rBfs.getPathLength() == rDijk.getPathLength() &&
                          rBfs.getPathLength() == rAstar.getPathLength() &&
                          rBfs.found && rDijk.found && rAstar.found);

            testGrid.loadPresetUnreachable();
            auto rUnreach = astar.findPath(testGrid, testGrid.getStart(), testGrid.getTarget());
            bool pass2 = (!rUnreach.found);

            testGrid.loadPresetWeightedSwamp();
            auto rBfsSwamp = bfs.findPath(testGrid, testGrid.getStart(), testGrid.getTarget());
            auto rDijkSwamp = dijkstra.findPath(testGrid, testGrid.getStart(), testGrid.getTarget());
            auto rAstarSwamp = astar.findPath(testGrid, testGrid.getStart(), testGrid.getTarget());
            bool pass3 = (rDijkSwamp.totalCost < rBfsSwamp.totalCost && rDijkSwamp.totalCost == rAstarSwamp.totalCost);

            testGrid.loadPresetOpen();
            rDijk = dijkstra.findPath(testGrid, testGrid.getStart(), testGrid.getTarget());
            rAstar = astar.findPath(testGrid, testGrid.getStart(), testGrid.getTarget());
            bool pass4 = (rAstar.nodesExplored < rDijk.nodesExplored);

            std::cout << "Test 1 (Optimality): " << (pass1 ? "PASS" : "FAIL") << "\n";
            std::cout << "Test 2 (Unreachable Target): " << (pass2 ? "PASS" : "FAIL") << "\n";
            std::cout << "Test 3 (Weighted Swamp Detour): " << (pass3 ? "PASS" : "FAIL") << "\n";
            std::cout << "Test 4 (A* Heuristic Pruning): " << (pass4 ? "PASS" : "FAIL") << "\n";

            if (pass1 && pass2 && pass3 && pass4) {
                std::cout << "ALL TESTS PASSED!\n";
                return 0;
            } else {
                std::cerr << "TESTS FAILED!\n";
                return 1;
            }
        } else if (arg == "--benchmark") {
            Grid grid(30, 15);
            grid.loadPresetObstacleWall();
            Benchmark benchmark;
            auto results = benchmark.run(grid);
            Benchmark::printResultsTable(results);
            return 0;
        }
    }

    // Launch interactive application
    Menu menu;
    menu.run();
    return 0;
}
