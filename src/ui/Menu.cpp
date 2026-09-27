#include "ui/Menu.hpp"
#include "ui/Visualizer.hpp"
#include "algorithms/BFS.hpp"
#include "algorithms/DFS.hpp"
#include "algorithms/Dijkstra.hpp"
#include "algorithms/AStar.hpp"

#include <iostream>
#include <limits>

namespace Pathfinding {

Menu::Menu()
    : m_grid(25, 15), m_animationDelayMs(30) {
    // Default starting map: Obstacle Wall
    m_grid.loadPresetObstacleWall();
}

void Menu::run() {
    Visualizer::initConsole();

    bool running = true;
    while (running) {
        displayMainMenu();
        int choice;
        std::cout << "\nEnter your choice [1-7]: ";
        if (!(std::cin >> choice)) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            continue;
        }

        switch (choice) {
            case 1:
                handleRunAlgorithm();
                break;
            case 2:
                handleBenchmark();
                break;
            case 3:
                handleSelectPreset();
                break;
            case 4:
                handleEditGrid();
                break;
            case 5:
                handleRunTestSuite();
                break;
            case 6: {
                std::cout << "Select animation delay in ms (current: " << m_animationDelayMs << "ms): ";
                int delay;
                if (std::cin >> delay && delay >= 0) {
                    m_animationDelayMs = delay;
                    std::cout << "Animation delay set to " << m_animationDelayMs << "ms.\n";
                }
                break;
            }
            case 7:
                std::cout << "\nThank you for using the Pathfinding Simulator! Goodbye.\n";
                running = false;
                break;
            default:
                std::cout << "Invalid choice. Please select 1-7.\n";
                break;
        }

        if (running && choice != 1 && choice != 2) {
            std::cout << "\nPress Enter to continue...";
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cin.get();
        }
    }
}

void Menu::displayMainMenu() {
    std::cout << "\n=======================================================\n";
    std::cout << "   GRID-BASED ROUTE PLANNING & PATHFINDING SIMULATOR  \n";
    std::cout << "=======================================================\n";
    Visualizer::renderLegend();
    std::cout << "\n[Current Grid Map (" << m_grid.getWidth() << "x" << m_grid.getHeight() << ")]:\n";
    Visualizer::renderGrid(m_grid);
    std::cout << "Start: " << m_grid.getStart() << " | Target: " << m_grid.getTarget()
              << " | Animation Delay: " << m_animationDelayMs << "ms\n";
    std::cout << "-------------------------------------------------------\n";
    std::cout << " 1. Run Single Pathfinding Algorithm (Interactive / Animated)\n";
    std::cout << " 2. Run Benchmark Comparison (All 4 Algorithms Side-by-Side)\n";
    std::cout << " 3. Select Map Preset (Corridor, Maze, Swamp, Obstacle...)\n";
    std::cout << " 4. Edit Grid (Add/Remove Walls, Move Start/Target)\n";
    std::cout << " 5. Run Automated Algorithm Verification Test Suite\n";
    std::cout << " 6. Adjust Animation Speed\n";
    std::cout << " 7. Exit\n";
    std::cout << "=======================================================\n";
}

void Menu::handleRunAlgorithm() {
    std::cout << "\n--- Select Algorithm ---\n";
    std::cout << "1. Breadth-First Search (BFS)\n";
    std::cout << "2. Depth-First Search (DFS)\n";
    std::cout << "3. Dijkstra's Algorithm (Uniform Cost Search)\n";
    std::cout << "4. A* Search (Manhattan Heuristic)\n";
    std::cout << "5. A* Search (Euclidean Heuristic)\n";
    std::cout << "Choice [1-5]: ";

    int choice;
    if (!(std::cin >> choice)) {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        return;
    }

    std::shared_ptr<PathfindingAlgorithm> algo;
    switch (choice) {
        case 1: algo = std::make_shared<BFSAlgorithm>(); break;
        case 2: algo = std::make_shared<DFSAlgorithm>(); break;
        case 3: algo = std::make_shared<DijkstraAlgorithm>(); break;
        case 4: algo = std::make_shared<AStarAlgorithm>(HeuristicType::Manhattan); break;
        case 5: algo = std::make_shared<AStarAlgorithm>(HeuristicType::Euclidean); break;
        default:
            std::cout << "Invalid algorithm choice.\n";
            return;
    }

    std::cout << "\nEnable animated step-by-step search visualization? (y/n): ";
    char animChoice;
    std::cin >> animChoice;
    bool animate = (animChoice == 'y' || animChoice == 'Y');

    executeAlgorithmWithAnimation(algo, animate);

    std::cout << "\nPress Enter to return to main menu...";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cin.get();
}

void Menu::executeAlgorithmWithAnimation(std::shared_ptr<PathfindingAlgorithm> algo, bool animate) {
    if (!animate || m_animationDelayMs == 0) {
        PathfindingResult result = algo->findPath(m_grid, m_grid.getStart(), m_grid.getTarget(), nullptr);
        Visualizer::renderResult(m_grid, result);
        return;
    }

    Grid animatedGrid = m_grid;
    animatedGrid.resetPathfinding();

    std::cout << "\n[Starting Search Animation for: " << algo->getName() << "]...\n";
    Visualizer::sleepMs(200);

    int stepCounter = 0;
    auto callback = [&](const Point& p, CellType type) {
        animatedGrid.getCell(p).setType(type);
        stepCounter++;
        // Refresh display every 1-2 steps for performance and smoothness
        if (stepCounter % 2 == 0) {
            Visualizer::clearScreen();
            std::cout << "Visualizing: " << algo->getName() << " | Exploring node: " << p << "\n";
            Visualizer::renderLegend();
            std::cout << "\n";
            Visualizer::renderGrid(animatedGrid);
            Visualizer::sleepMs(m_animationDelayMs);
        }
    };

    PathfindingResult result = algo->findPath(m_grid, m_grid.getStart(), m_grid.getTarget(), callback);

    Visualizer::clearScreen();
    Visualizer::renderResult(m_grid, result);
}

void Menu::handleBenchmark() {
    std::cout << "\nRunning benchmark across all algorithms on current grid...\n";
    auto results = m_benchmark.run(m_grid);
    Benchmark::printResultsTable(results);

    std::cout << "\nPress Enter to return to main menu...";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cin.get();
}

void Menu::handleSelectPreset() {
    std::cout << "\n--- Select Map Preset ---\n";
    std::cout << "1. Open Field (No obstacles)\n";
    std::cout << "2. Narrow Corridor / Labyrinth (S-curves)\n";
    std::cout << "3. Obstacle Wall Barrier (Central wall with gaps)\n";
    std::cout << "4. Random Procedural Maze\n";
    std::cout << "5. Weighted Swamp / Mud Terrain (Dijkstra/A* detour demo)\n";
    std::cout << "6. Unreachable Target (Enclosed in walls)\n";
    std::cout << "Choice [1-6]: ";

    int choice;
    if (!(std::cin >> choice)) {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        return;
    }

    switch (choice) {
        case 1: m_grid.loadPresetOpen(); break;
        case 2: m_grid.loadPresetCorridor(); break;
        case 3: m_grid.loadPresetObstacleWall(); break;
        case 4: m_grid.loadPresetMaze(); break;
        case 5: m_grid.loadPresetWeightedSwamp(); break;
        case 6: m_grid.loadPresetUnreachable(); break;
        default:
            std::cout << "Invalid preset.\n";
            return;
    }

    std::cout << "Preset loaded successfully!\n";
}

void Menu::handleEditGrid() {
    std::cout << "\n--- Edit Grid ---\n";
    std::cout << "1. Toggle Wall at coordinate (x, y)\n";
    std::cout << "2. Move Start position (x, y)\n";
    std::cout << "3. Move Target position (x, y)\n";
    std::cout << "4. Clear all walls and reset to open grid\n";
    std::cout << "Choice [1-4]: ";

    int choice;
    if (!(std::cin >> choice)) {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        return;
    }

    if (choice == 1) {
        int x, y;
        std::cout << "Enter (x y) to toggle wall: ";
        if (std::cin >> x >> y) {
            if (m_grid.toggleWall(Point(x, y))) {
                std::cout << "Wall toggled at (" << x << ", " << y << ").\n";
            } else {
                std::cout << "Cannot place wall at (" << x << ", " << y << ") - out of bounds or on Start/Target.\n";
            }
        }
    } else if (choice == 2) {
        int x, y;
        std::cout << "Enter new Start (x y): ";
        if (std::cin >> x >> y) {
            if (m_grid.setStart(Point(x, y))) {
                std::cout << "Start moved to (" << x << ", " << y << ").\n";
            } else {
                std::cout << "Invalid position for Start.\n";
            }
        }
    } else if (choice == 3) {
        int x, y;
        std::cout << "Enter new Target (x y): ";
        if (std::cin >> x >> y) {
            if (m_grid.setTarget(Point(x, y))) {
                std::cout << "Target moved to (" << x << ", " << y << ").\n";
            } else {
                std::cout << "Invalid position for Target.\n";
            }
        }
    } else if (choice == 4) {
        m_grid.clearAll();
        std::cout << "All walls cleared.\n";
    }
}

void Menu::handleRunTestSuite() {
    std::cout << "\n[Running Automated Algorithm Verification Suite]...\n";

    // Test 1: Open field optimality
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
    auto rDfs = dfs.findPath(testGrid, start, target);
    auto rDijk = dijkstra.findPath(testGrid, start, target);
    auto rAstar = astar.findPath(testGrid, start, target);

    std::cout << "Test 1: Open Field Optimal Path Check\n";
    std::cout << "  BFS Path Length      : " << rBfs.getPathLength() << " (Found: " << rBfs.found << ")\n";
    std::cout << "  Dijkstra Path Length : " << rDijk.getPathLength() << " (Found: " << rDijk.found << ")\n";
    std::cout << "  A* Path Length       : " << rAstar.getPathLength() << " (Found: " << rAstar.found << ")\n";
    std::cout << "  DFS Path Length      : " << rDfs.getPathLength() << " (Found: " << rDfs.found << ")\n";

    bool pass1 = (rBfs.getPathLength() == rDijk.getPathLength() &&
                  rBfs.getPathLength() == rAstar.getPathLength() &&
                  rBfs.found && rDijk.found && rAstar.found);
    std::cout << "  Result: " << (pass1 ? "\033[1;32m[PASS]\033[0m" : "\033[1;31m[FAIL]\033[0m")
              << " - BFS, Dijkstra, and A* found equal optimal length!\n\n";

    // Test 2: Unreachable target check
    testGrid.loadPresetUnreachable();
    auto rUnreach = astar.findPath(testGrid, testGrid.getStart(), testGrid.getTarget());
    bool pass2 = (!rUnreach.found);
    std::cout << "Test 2: Unreachable Target Handling\n";
    std::cout << "  Target trapped in walls -> Path found? " << (rUnreach.found ? "YES" : "NO") << "\n";
    std::cout << "  Result: " << (pass2 ? "\033[1;32m[PASS]\033[0m" : "\033[1;31m[FAIL]\033[0m")
              << " - Correctly reported no path available!\n\n";

    // Test 3: Weighted Swamp Terrain Check (Dijkstra/A* detour vs BFS hop-count)
    testGrid.loadPresetWeightedSwamp();
    auto rBfsSwamp = bfs.findPath(testGrid, testGrid.getStart(), testGrid.getTarget());
    auto rDijkSwamp = dijkstra.findPath(testGrid, testGrid.getStart(), testGrid.getTarget());
    auto rAstarSwamp = astar.findPath(testGrid, testGrid.getStart(), testGrid.getTarget());

    std::cout << "Test 3: Weighted Terrain Cost Minimization\n";
    std::cout << "  BFS (Unweighted hop-count) Cost : " << rBfsSwamp.totalCost << " (Steps: " << rBfsSwamp.getPathLength() << ")\n";
    std::cout << "  Dijkstra (Cost-aware) Cost      : " << rDijkSwamp.totalCost << " (Steps: " << rDijkSwamp.getPathLength() << ")\n";
    std::cout << "  A* (Cost-aware) Cost            : " << rAstarSwamp.totalCost << " (Steps: " << rAstarSwamp.getPathLength() << ")\n";

    bool pass3 = (rDijkSwamp.totalCost < rBfsSwamp.totalCost && rDijkSwamp.totalCost == rAstarSwamp.totalCost);
    std::cout << "  Result: " << (pass3 ? "\033[1;32m[PASS]\033[0m" : "\033[1;31m[FAIL]\033[0m")
              << " - Dijkstra and A* found cheaper detour than BFS!\n\n";

    // Test 4: Efficiency (A* explores fewer nodes than Dijkstra in open field)
    testGrid.loadPresetOpen();
    rDijk = dijkstra.findPath(testGrid, testGrid.getStart(), testGrid.getTarget());
    rAstar = astar.findPath(testGrid, testGrid.getStart(), testGrid.getTarget());
    std::cout << "Test 4: Heuristic Search Efficiency (Nodes Explored)\n";
    std::cout << "  Dijkstra Nodes Explored : " << rDijk.nodesExplored << "\n";
    std::cout << "  A* Nodes Explored       : " << rAstar.nodesExplored << "\n";
    bool pass4 = (rAstar.nodesExplored < rDijk.nodesExplored);
    std::cout << "  Result: " << (pass4 ? "\033[1;32m[PASS]\033[0m" : "\033[1;31m[FAIL]\033[0m")
              << " - A* heuristic pruned exploration compared to Dijkstra!\n\n";

    if (pass1 && pass2 && pass3 && pass4) {
        std::cout << "\033[1;32mALL 4 TEST CASES PASSED SUCCESSFULLY!\033[0m\n";
    } else {
        std::cout << "\033[1;31mSOME TEST CASES FAILED!\033[0m\n";
    }
}

} // namespace Pathfinding
