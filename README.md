# Grid-Based Route Planning & Pathfinding Simulator

An interactive, object-oriented C++ route planning and pathfinding simulator implementing **Breadth-First Search (BFS)**, **Depth-First Search (DFS)**, **Dijkstra's Algorithm**, and **A* Search (Manhattan & Euclidean)** with real-time terminal visualization and performance benchmarking.

---

## Features

- **Object-Oriented Architecture (OOP)**:
  - **Strategy Pattern**: `PathfindingAlgorithm` interface allows interchangeable algorithms at runtime without changing client code.
  - **Encapsulated Models**: `Grid`, `Cell`, and `Point` classes cleanly isolate grid logic, distance metrics, bounds checking, and cell weights.
  - **Observer/Callback Mechanism**: `StepCallback` enables real-time animated search progress visualization without tightly coupling algorithms to the display engine.
- **Multiple Graph Algorithms**:
  - **Breadth-First Search (BFS)**: FIFO queue search guaranteeing minimum step count in unweighted environments.
  - **Depth-First Search (DFS)**: Stack/LIFO search demonstrating branch-first exploration and sub-optimal paths.
  - **Dijkstra's Algorithm**: Min-priority queue search guaranteeing the lowest cost in weighted grids.
  - **A* Search**: Informed heuristic search ($f(n) = g(n) + h(n)$) with Manhattan and Euclidean distance heuristics.
- **Real-Time ANSI Visualizer**:
  - Color-coded grid: Green `[S]` Start, Red `[E]` Target, Gray `[#]` Walls, Yellow `[*]` Path, Cyan `[.]` Visited, Magenta `[?]` Frontier, Yellow `[~]` Swamp/Mud.
  - Step-by-step search animation with configurable delays.
- **Side-by-Side Comparative Benchmarking**:
  - Comprehensive metrics: Path Found, Steps, Total Cost, Visited Nodes, Execution Time (µs), and Optimality analysis.
- **Map Presets & Interactive Editor**:
  - Presets: *Open Field*, *Narrow Corridor*, *Obstacle Wall*, *Procedural Maze*, *Weighted Swamp Terrain*, and *Unreachable Target*.
  - In-terminal editor to move start/target and toggle walls.

---

## Project Structure

```
route-planning/
├── include/
│   ├── core/
│   │   ├── Point.hpp              # 2D coordinates & distance metrics
│   │   ├── Cell.hpp               # Grid unit state & terrain cost
│   │   └── Grid.hpp               # 2D grid container & map presets
│   ├── algorithms/
│   │   ├── PathfindingResult.hpp  # Path metrics & telemetry
│   │   ├── PathfindingAlgorithm.hpp # Strategy base class
│   │   ├── BFS.hpp                # Breadth-First Search
│   │   ├── DFS.hpp                # Depth-First Search
│   │   ├── Dijkstra.hpp           # Dijkstra's Algorithm
│   │   └── AStar.hpp              # A* Search
│   ├── benchmark/
│   │   └── Benchmark.hpp          # Comparative benchmark engine
│   └── ui/
│       ├── Visualizer.hpp         # ANSI color renderer & animation
│       └── Menu.hpp               # Interactive CLI menu
├── src/
│   ├── core/
│   ├── algorithms/
│   ├── benchmark/
│   ├── ui/
│   └── main.cpp                   # Application entry point
├── Makefile                       # MinGW compilation script
└── README.md
```

---

## Build & Execution

### Prerequisites
- GCC / MinGW (`g++` supporting C++17)
- `mingw32-make` or standard `make`

### Building the Simulator
```powershell
mingw32-make
```

### Running the Interactive Simulator
```powershell
.\simulator.exe
```

### Running the Automated Test Suite
```powershell
.\simulator.exe --test
```

### Running Headless Benchmark
```powershell
.\simulator.exe --benchmark
```

---

## Algorithm Comparison Summary

| Algorithm | Data Structure | Weighted Graphs? | Shortest Path Guarantee? | Explored Nodes Efficiency |
| :--- | :--- | :---: | :---: | :--- |
| **BFS** | `std::queue` (FIFO) | No (Unweighted only) | **Yes** (in step count) | Moderate (Uniform expansion) |
| **DFS** | `std::stack` (LIFO) | No | **No** (often jagged/long) | Low/Unpredictable |
| **Dijkstra** | `std::priority_queue` (Min-heap) | **Yes** | **Yes** (in total cost) | High cost (Evaluates all lower-cost paths) |
| **A\*** | `std::priority_queue` (Min-heap) | **Yes** | **Yes** (with admissible heuristic) | **Highest** (Pruned toward target) |
