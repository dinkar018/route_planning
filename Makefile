CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -Iinclude
TARGET = simulator.exe

SRCS = src/core/Cell.cpp \
       src/core/Grid.cpp \
       src/algorithms/BFS.cpp \
       src/algorithms/DFS.cpp \
       src/algorithms/Dijkstra.cpp \
       src/algorithms/AStar.cpp \
       src/benchmark/Benchmark.cpp \
       src/ui/Visualizer.cpp \
       src/ui/Menu.cpp \
       src/main.cpp

OBJS = $(SRCS:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

test: $(TARGET)
	./$(TARGET) --test

benchmark: $(TARGET)
	./$(TARGET) --benchmark

clean:
	-del /Q /S src\*.o 2>nul
	-del /Q $(TARGET) 2>nul

.PHONY: all clean test benchmark
