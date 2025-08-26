# Makefile for eyeDoctor tests

# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O3 -fopenmp
INCLUDES = -I/usr/local/include/eigen3 -I/usr/local/include

# Source files
TEST_SOURCES = testEyeDoctor.cpp testEyeDoctorIntegration.cpp
TEST_OBJECTS = $(TEST_SOURCES:.cpp=.o)

# Test executables
TEST_EXEC = testEyeDoctor
INTEGRATION_EXEC = testEyeDoctorIntegration

# Default target
all: $(TEST_EXEC) $(INTEGRATION_EXEC)

# Build test executable
$(TEST_EXEC): testEyeDoctor.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(INCLUDES)

# Build integration test executable
$(INTEGRATION_EXEC): testEyeDoctorIntegration.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(INCLUDES)

# Compile test objects
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Clean
clean:
	rm -f $(TEST_OBJECTS) $(TEST_EXEC) $(INTEGRATION_EXEC)

# Run basic tests
test: $(TEST_EXEC)
	./$(TEST_EXEC)

# Run integration tests
test-integration: $(INTEGRATION_EXEC)
	./$(INTEGRATION_EXEC)

# Run all tests
test-all: test test-integration

.PHONY: all clean test test-integration test-all
