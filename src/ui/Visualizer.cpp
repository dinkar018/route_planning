#include "ui/Visualizer.hpp"
#include <iostream>
#include <thread>
#include <chrono>

#ifdef _WIN32
#include <windows.h>
#endif

namespace Pathfinding {

void Visualizer::initConsole() {
#ifdef _WIN32
    HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
    if (hOut != INVALID_HANDLE_VALUE) {
        DWORD dwMode = 0;
        if (GetConsoleMode(hOut, &dwMode)) {
            dwMode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;
            SetConsoleMode(hOut, dwMode);
        }
    }
#endif
}

void Visualizer::clearScreen() {
    // ANSI code to clear screen and move cursor to (1,1)
    std::cout << "\033[2J\033[H" << std::flush;
}

void Visualizer::sleepMs(int milliseconds) {
#ifdef _WIN32
    Sleep(milliseconds);
#else
    // POSIX fallback if compiled on Linux
    struct timespec ts;
    ts.tv_sec = milliseconds / 1000;
    ts.tv_nsec = (milliseconds % 1000) * 1000000;
    nanosleep(&ts, NULL);
#endif
}

void Visualizer::renderLegend() {
    std::cout << "Legend: "
              << "\033[1;32m[S] Start\033[0m  "
              << "\033[1;31m[E] Target\033[0m  "
              << "\033[1;90m[#] Wall\033[0m  "
              << "\033[1;33m[*] Path\033[0m  "
              << "\033[36m[.] Visited\033[0m  "
              << "\033[33m[~] Swamp(cost:5)\033[0m\n";
}

void Visualizer::renderGrid(const Grid& grid) {
    int w = grid.getWidth();
    int h = grid.getHeight();

    // Column numbers header
    std::cout << "   ";
    for (int x = 0; x < w; ++x) {
        std::cout << (x % 10);
    }
    std::cout << "\n";

    for (int y = 0; y < h; ++y) {
        // Row number
        if (y < 10) std::cout << " " << y << " ";
        else std::cout << y << " ";

        for (int x = 0; x < w; ++x) {
            const Cell& cell = grid.getCell(x, y);
            std::cout << cell.getAnsiColor() << cell.toChar() << "\033[0m";
        }
        std::cout << "\n";
    }
}

void Visualizer::renderResult(const Grid& grid, const PathfindingResult& result) {
    // Create a copy of the grid to overlay visited nodes and path
    Grid displayGrid = grid;
    displayGrid.resetPathfinding();

    // Mark visited nodes
    for (const Point& p : result.visitedOrder) {
        if (p != displayGrid.getStart() && p != displayGrid.getTarget()) {
            displayGrid.getCell(p).setType(CellType::Visited);
        }
    }

    // Mark path nodes
    if (result.found) {
        for (const Point& p : result.path) {
            if (p != displayGrid.getStart() && p != displayGrid.getTarget()) {
                displayGrid.getCell(p).setType(CellType::Path);
            }
        }
    }

    std::cout << "\n=======================================================\n";
    std::cout << " Result: " << result.algorithmName << "\n";
    std::cout << "=======================================================\n";
    renderLegend();
    std::cout << "\n";
    renderGrid(displayGrid);
    std::cout << "-------------------------------------------------------\n";
    std::cout << " Status:        " << (result.found ? "\033[1;32mPATH FOUND\033[0m" : "\033[1;31mNO PATH FOUND\033[0m") << "\n";
    if (result.found) {
        std::cout << " Path Length:   " << result.getPathLength() << " steps\n";
        std::cout << " Total Cost:    " << result.totalCost << "\n";
    }
    std::cout << " Nodes Visited: " << result.nodesExplored << "\n";
    std::cout << " Runtime:       " << result.durationMicroseconds << " us ("
              << (result.durationMicroseconds / 1000.0) << " ms)\n";
    std::cout << "=======================================================\n";
}

} // namespace Pathfinding
