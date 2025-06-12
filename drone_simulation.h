// drone_simulation.h
#ifndef DRONE_SIMULATION_H
#define DRONE_SIMULATION_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/wait.h>
#include <signal.h>
#include <time.h>
#include <stdarg.h>

// --- Constants ---
#define MAX_DRONES 10
#define MAX_INSTRUCTIONS 50
#define MAX_INSTRUCTION_LEN 10 // Max length of an instruction string like "FORWARD"
#define BUFFER_SIZE 256        // General purpose buffer size for reading lines
#define REPORT_FILENAME "simulation_report.txt"
#define COLLISION_THRESHOLD 3  // Max allowed collisions before simulation terminates
#define MAX_TIME_STEPS 100     // Safety limit for simulation loops and array sizes

// --- Enumerations ---

// Defines the types of commands a drone can execute
typedef enum {
    CMD_UP, CMD_DOWN, CMD_LEFT, CMD_RIGHT,
    CMD_FORWARD, CMD_BACKWARD, CMD_SHAKE, CMD_ROTATE,
    CMD_UNKNOWN, // Represents an unrecognized command
    CMD_END      // Placeholder, not typically used as a parsed command
} CommandType;

// Defines message types sent from the parent process to child drone processes
typedef enum {
    MSG_TYPE_PROCEED,  // Command to execute the next instruction
    MSG_TYPE_TERMINATE // Command to gracefully shut down
} ParentToChildMessageType;

// --- Structures ---

// Message structure for parent-to-child communication
typedef struct {
    ParentToChildMessageType type;
} ParentToChildMessage;

// Message structure for child-to-parent communication, updating drone status
typedef struct {
    int drone_id;                   // Unique ID of the reporting drone
    int new_x, new_y, new_z;        // Drone's new 3D coordinates
    int finished;                   // Flag: 1 if drone completed all instructions, 0 otherwise
    int instruction_executed_index; // Index of the instruction just performed
} DroneUpdate;

// Structure to log collision details for the report
typedef struct {
    int time_step;                  // Simulation step number when collision occurred
    time_t timestamp;               // Real-world timestamp of the collision
    int drone_id1, drone_id2;       // IDs of the two drones involved
    int x, y, z;                    // 3D coordinates of the collision point
} CollisionEvent;

// Structure representing a single drone's state and flight plan (managed by parent)
typedef struct {
    int id;                         // Unique identifier for the drone
    int x, y, z;                    // Current 3D coordinates
    CommandType instructions[MAX_INSTRUCTIONS]; // Array of commands for the drone
    int num_instructions;           // Total number of instructions in its plan
    int current_instruction_index_tracker; // Parent's tracker of the next instruction to be executed
    int active;                     // Status: 1 if still flying, 0 if finished or deactivated
    pid_t pid;                      // Process ID of the child process managing this drone
    int pipe_to_child_write_fd;     // File descriptor for writing commands to the child
    int pipe_from_child_read_fd;    // File descriptor for reading updates from the child
} Drone;

// --- Global Variables (declared as extern, defined in main_controller.c or reporting.c) ---
extern Drone sim_drones[MAX_DRONES]; // Array holding all drone states
extern int num_sim_drones;           // Current number of drones in the simulation
extern FILE *report_file;            // File pointer for the simulation report
extern int total_collisions_count;   // Counter for all detected collisions
extern CollisionEvent collision_log[MAX_DRONES * MAX_TIME_STEPS]; // Log of all collisions
extern int collision_log_index;      // Current index for the collision_log array
extern int drone_positions_history[MAX_TIME_STEPS][MAX_DRONES][3]; // History of drone positions

// --- Function Prototypes ---

// csv_parser.c
CommandType string_to_command(const char* str);
const char* command_to_string(CommandType cmd); // For readable output
int load_drones_from_csv(const char* filename, Drone drones_arr[], int* drone_count_ptr);

// drone_logic.c
void drone_child_process(Drone initial_drone_config, int pipe_read_from_parent_fd, int pipe_write_to_parent_fd);
void signal_handler_collision_child(int signum); // Renamed for clarity

// reporting.c
int init_report(const char* filename);
void log_to_report(const char* format, ...);
void log_initial_drone_states_to_report(void);
void log_time_step_header_to_report(int time_step);
void log_drone_update_to_report(const Drone* drone, const DroneUpdate* update, CommandType cmd_type);
void log_drone_finish_to_report(const Drone* drone);
void log_error_to_report(const char* error_message);
void log_collision_to_report(int drone_id1, int drone_id2, int x, int y, int z, int time_step);
void log_simulation_summary_to_report(int final_time_step, int simulation_failed_status);
void close_report(void);

// main_controller.c (main is implicitly declared)

#endif // DRONE_SIMULATION_H