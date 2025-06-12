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

// --- New includes for Shared Memory and Semaphores ---
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <semaphore.h>
// ADDED: Include for POSIX threads
#include <pthread.h>

// --- Constants ---
#define MAX_DRONES 10
#define MAX_INSTRUCTIONS 50
#define MAX_INSTRUCTION_LEN 10
#define BUFFER_SIZE 256
#define REPORT_FILENAME "simulation_report.txt"
#define COLLISION_THRESHOLD 3
#define MAX_TIME_STEPS 100

// --- Shared Memory and Semaphore Naming ---
#define SHM_NAME "/drone_sim_shm"
#define SEM_PARENT_PREFIX "/sim_parent_sem_" // Parent waits on this
#define SEM_CHILD_PREFIX "/sim_child_sem_"   // Child waits on this

// --- Enumerations ---

typedef enum {
    CMD_UP, CMD_DOWN, CMD_LEFT, CMD_RIGHT,
    CMD_FORWARD, CMD_BACKWARD, CMD_SHAKE, CMD_ROTATE,
    CMD_UNKNOWN,
    CMD_END
} CommandType;

// --- Structures ---

// Structure representing a single drone's state in SHARED MEMORY
typedef struct {
    pid_t pid;
    int id;
    int x, y, z;
    int finished;
    int active;
    int instruction_executed_index;
    int terminate_flag;
} DroneSharedState;

// Layout of the entire shared memory segment
typedef struct {
    int total_collisions_count;
    int simulation_running;
    DroneSharedState drones[MAX_DRONES];
} SharedMemoryLayout;

// Structure for logging collision details for the report
typedef struct {
    int time_step;
    time_t timestamp;
    int drone_id1, drone_id2;
    int x, y, z;
} CollisionEvent;

// Structure representing a single drone's configuration
typedef struct {
    int id;
    int initial_x, initial_y, initial_z;
    CommandType instructions[MAX_INSTRUCTIONS];
    int num_instructions;
    pid_t pid;
    sem_t *sem_parent_can_read;
    sem_t *sem_child_can_act;
} Drone;


// --- Global Variables ---
extern Drone sim_drones[MAX_DRONES];
extern int num_sim_drones;
extern FILE *report_file;
extern int total_collisions_count;
extern CollisionEvent collision_log[MAX_DRONES * MAX_TIME_STEPS];
extern int collision_log_index;
extern int drone_positions_history[MAX_TIME_STEPS][MAX_DRONES][3];
extern SharedMemoryLayout *shared_mem;

// --- Function Prototypes ---

// csv_parser.c
CommandType string_to_command(const char* str);
const char* command_to_string(CommandType cmd);
int load_drones_from_csv(const char* filename, Drone drones_arr[], int* drone_count_ptr);

// drone_logic.c
void drone_child_process(int drone_index, Drone initial_drone_config);

// reporting.c
int init_report(const char* filename);
void log_to_report(const char* format, ...);
void log_initial_drone_states_to_report(void);
void log_time_step_header_to_report(int time_step);
void log_drone_update_to_report(const DroneSharedState* update, CommandType cmd_type);
void log_drone_finish_to_report(const DroneSharedState* update);
void log_error_to_report(const char* error_message);
void log_collision_to_report(int drone_id1, int drone_id2, int x, int y, int z, int time_step);
void log_simulation_summary_to_report(int final_time_step, int simulation_failed_status);
void close_report(void);

#endif // DRONE_SIMULATION_H