CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2
LDFLAGS = -lm

CXXFLAGS +=	-I/usr/include

SOURCES = golf-simulation-3d.cpp
EXECUTABLE = build/golf_simulation

all: $(EXECUTABLE)

$(EXECUTABLE): $(SOURCES)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -f $(EXECUTABLE) simulation_results.json

run: $(EXECUTABLE)
	./$(EXECUTABLE)

visualise: run
	python3 golf_visualisation.py --skip-simulation
