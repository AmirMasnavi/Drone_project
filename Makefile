# Makefile for Drone Simulation Project

# Compiler and flags
CC = gcc
CFLAGS = -Wall -Wextra -g -std=c11
LDFLAGS = -lcurl # <--- ADDED for libcurl

# Source files
SRCS = main_controller.c csv_parser.c drone_logic.c reporting.c ui_display.c

# Object files (derived from SRCS)
OBJS = $(SRCS:.c=.o)

# Executable name
TARGET = drone_simulator

# Default target: build the executable
all: $(TARGET)

# Rule to link the executable from object files
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $(TARGET) $(LDFLAGS)

# Rule to compile .c files into .o files
%.o: %.c drone_simulation.h
	$(CC) $(CFLAGS) -c $< -o $@

# Specific dependencies can be listed if needed, e.g.:
ui_display.o: ui_display.c ui_display.h drone_simulation.h
	$(CC) $(CFLAGS) -c $< -o $@

# Target to clean up build files
clean:
	rm -f $(OBJS) $(TARGET) simulation_report.txt

# Phony targets are not actual files
.PHONY: all clean


# Makefile for Drone Simulation Project

# Compiler and flags
CC = gcc
CFLAGS = -Wall -Wextra -g -std=c11
LDFLAGS = -lcurl

# Source files for main application
APP_SRCS = main_controller.c csv_parser.c drone_logic.c reporting.c ui_display.c
APP_OBJS = $(APP_SRCS:.c=.o)
TARGET = drone_simulator

# Test source files (assuming they are in a 'tests' subdirectory)
TEST_CHILD_LOGIC_SRC = tests/test_child_logic.c drone_logic.c # Needs drone_logic.o
TEST_CHILD_LOGIC_OBJS = tests/test_child_logic.o drone_logic.o
TEST_CHILD_LOGIC_TARGET = tests/test_child_logic

TEST_SIGNAL_HANDLING_SRC = tests/test_signal_handling.c drone_logic.c
TEST_SIGNAL_HANDLING_OBJS = tests/test_signal_handling.o drone_logic.o
TEST_SIGNAL_HANDLING_TARGET = tests/test_signal_handling

TEST_COLLISION_DETECTION_SRC = tests/test_collision_detection.c # This might need reporting.o if log_collision is called
TEST_COLLISION_DETECTION_OBJS = tests/test_collision_detection.o # And reporting.o
TEST_COLLISION_DETECTION_TARGET = tests/test_collision_detection


# Default target: build the main application
all: $(TARGET)

$(TARGET): $(APP_OBJS)
	$(CC) $(CFLAGS) $(APP_OBJS) -o $(TARGET) $(LDFLAGS)

# Rule to compile .c files into .o files (generic)
# This rule needs to be careful about header locations if tests are in subdir
# Assuming drone_simulation.h is in parent dir relative to test .c files
%.o: %.c
	$(CC) $(CFLAGS) -I.. -c $< -o $@

# Specific rule for source files in the current directory (main app)
main_controller.o: main_controller.c drone_simulation.h ui_display.h
	$(CC) $(CFLAGS) -c main_controller.c -o main_controller.o
csv_parser.o: csv_parser.c drone_simulation.h
	$(CC) $(CFLAGS) -c csv_parser.c -o csv_parser.o
drone_logic.o: drone_logic.c drone_simulation.h
	$(CC) $(CFLAGS) -c drone_logic.c -o drone_logic.o
reporting.o: reporting.c drone_simulation.h
	$(CC) $(CFLAGS) -c reporting.c -o reporting.o
ui_display.o: ui_display.c ui_display.h drone_simulation.h
	$(CC) $(CFLAGS) -c ui_display.c -o ui_display.o


# Rules for test executables
tests/test_child_logic.o: tests/test_child_logic.c drone_simulation.h
	$(CC) $(CFLAGS) -I.. -c tests/test_child_logic.c -o tests/test_child_logic.o

$(TEST_CHILD_LOGIC_TARGET): $(TEST_CHILD_LOGIC_OBJS)
	$(CC) $(CFLAGS) $(TEST_CHILD_LOGIC_OBJS) -o $(TEST_CHILD_LOGIC_TARGET) $(LDFLAGS)

tests/test_signal_handling.o: tests/test_signal_handling.c drone_simulation.h
	$(CC) $(CFLAGS) -I.. -c tests/test_signal_handling.c -o tests/test_signal_handling.o

$(TEST_SIGNAL_HANDLING_TARGET): $(TEST_SIGNAL_HANDLING_OBJS)
	$(CC) $(CFLAGS) $(TEST_SIGNAL_HANDLING_OBJS) -o $(TEST_SIGNAL_HANDLING_TARGET) $(LDFLAGS)

tests/test_collision_detection.o: tests/test_collision_detection.c drone_simulation.h
	$(CC) $(CFLAGS) -I.. -c tests/test_collision_detection.c -o tests/test_collision_detection.o

$(TEST_COLLISION_DETECTION_TARGET): $(TEST_COLLISION_DETECTION_OBJS)
	$(CC) $(CFLAGS) $(TEST_COLLISION_DETECTION_OBJS) -o $(TEST_COLLISION_DETECTION_TARGET) $(LDFLAGS)


# Target to run all tests
test: $(TEST_CHILD_LOGIC_TARGET) $(TEST_SIGNAL_HANDLING_TARGET) $(TEST_COLLISION_DETECTION_TARGET)
	@echo "--- Running Child Logic Tests ---"
	@$(TEST_CHILD_LOGIC_TARGET)
	@echo "--- Running Signal Handling Tests ---"
	@$(TEST_SIGNAL_HANDLING_TARGET)
	@echo "--- Running Collision Detection Tests ---"
	@$(TEST_COLLISION_DETECTION_TARGET)
	@echo "--- All Tests Complete ---"


clean:
	rm -f $(APP_OBJS) $(TARGET) \
	tests/*.o tests/test_child_logic tests/test_signal_handling tests/test_collision_detection \
	simulation_report.txt 

.PHONY: all clean test