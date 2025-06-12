// reporting.c
#include "drone_simulation.h"
#include <stdarg.h> // For va_list, va_start, va_end

// Global definition for report_file (declared extern in .h)
FILE *report_file = NULL;
// Global definitions for collision logging (declared extern in .h)
CollisionEvent collision_log[MAX_DRONES * MAX_TIME_STEPS];
int collision_log_index = 0;
SharedMemoryLayout *shared_mem; // This is needed to access drone states



// Initializes the report file. Returns 1 on success, 0 on failure.
int init_report(const char* filename) {
    report_file = fopen(filename, "w");
    if (!report_file) {
        perror("REPORTING: Error opening report file");
        return 0;
    }
    fprintf(report_file, "==== Simulation Report ====\n");
    time_t now = time(NULL);
    fprintf(report_file, "Report generated on: %s", ctime(&now)); // ctime adds newline
    fprintf(report_file, "===========================\n\n");
    fflush(report_file);
    return 1;
}

// Generic function to log a formatted message to the report file.
void log_to_report(const char* format, ...) { // Designed for variable arguments
    if (!report_file) return;
    va_list args;
    va_start(args, format);
    vfprintf(report_file, format, args); // Uses vfprintf
    va_end(args);
    fflush(report_file);
}

// Logs the initial states of all loaded drones.
void log_initial_drone_states_to_report(void) {
    if (!report_file) return;
    fprintf(report_file, "Initial Drone States (Loaded %d drones):\n", num_sim_drones);
    for (int i = 0; i < num_sim_drones; ++i) {
        // Here we use the config struct, as shared_mem is also initialized from this
        fprintf(report_file, "  Drone ID %d: Start Pos (%d, %d, %d), Instructions: %d\n",
                sim_drones[i].id, sim_drones[i].initial_x, sim_drones[i].initial_y, sim_drones[i].initial_z,
                sim_drones[i].num_instructions);
    }
    fprintf(report_file, "---------------------------------------\n\n");
    fflush(report_file);
}

// Logs the header for a new time step.
void log_time_step_header_to_report(int time_step) {
    if (!report_file) return;
    fprintf(report_file, "--- Time Step %d ---\n", time_step);
    fflush(report_file);
}

// Logs an update from a drone (movement or status change).
void log_drone_update_to_report(const DroneSharedState* update, CommandType cmd_type) {
    if (!report_file) return;
    fprintf(report_file, "  Drone ID %d: Pos (%d, %d, %d), Executed Instr %d (%s)\n",
            update->id, update->x, update->y, update->z,
            update->instruction_executed_index, command_to_string(cmd_type));
    fflush(report_file);
}

// Logs that a drone has finished its instructions.
void log_drone_finish_to_report(const DroneSharedState* update) {
    if (!report_file) return;
    fprintf(report_file, "  Drone ID %d: Pos (%d, %d, %d) - FINISHED flight plan.\n",
            update->id, update->x, update->y, update->z);
    fflush(report_file);
}


// Logs an error message to the report.
void log_error_to_report(const char* error_message) {
    if (!report_file) return;
    fprintf(report_file, "ERROR: %s\n", error_message);
    fflush(report_file);
}

// Logs a detected collision event.
void log_collision_to_report(int drone_id1, int drone_id2, int x, int y, int z, int time_step) {
    if (!report_file) return;
    time_t now = time(NULL);
    char time_str_buffer[30];
    strftime(time_str_buffer, sizeof(time_str_buffer), "%Y-%m-%d %H:%M:%S", localtime(&now));

    fprintf(report_file, "  COLLISION! Drones %d and %d at (%d, %d, %d). Timestamp: %s\n",
            drone_id1, drone_id2, x, y, z, time_str_buffer);
    fflush(report_file);

    // Also add to the internal collision log for summary
    if (collision_log_index < (MAX_DRONES * MAX_TIME_STEPS)) {
        collision_log[collision_log_index].time_step = time_step;
        collision_log[collision_log_index].timestamp = now;
        collision_log[collision_log_index].drone_id1 = drone_id1;
        collision_log[collision_log_index].drone_id2 = drone_id2;
        collision_log[collision_log_index].x = x;
        collision_log[collision_log_index].y = y;
        collision_log[collision_log_index].z = z;
        collision_log_index++;
    }
}


// Logs the final summary of the simulation.
// simulation_failed_status: 0=Passed, 1=Completed with Collisions, 2=Failed (Threshold), 3=Failed (Incomplete)
void log_simulation_summary_to_report(int final_time_step, int simulation_status_code) {
    if (!report_file) return;
    fprintf(report_file, "\n==== Simulation Summary ====\n");
    fprintf(report_file, "Total Drones Simulated: %d\n", num_sim_drones);
    fprintf(report_file, "Total Time Steps Executed: %d\n", final_time_step > 0 ? final_time_step - 1 : 0);
    fprintf(report_file, "Total Collisions Detected: %d\n", total_collisions_count);

    fprintf(report_file, "\nCollision Event Log (%d entries):\n", collision_log_index);
    if (collision_log_index == 0) {
        fprintf(report_file, "  No collisions occurred during the simulation.\n");
    } else {
        for (int i = 0; i < collision_log_index; ++i) {
            char time_str_buffer[30];
            strftime(time_str_buffer, sizeof(time_str_buffer), "%Y-%m-%d %H:%M:%S", localtime(&collision_log[i].timestamp));
            fprintf(report_file, "  Event %d: Time Step %d, Drones %d & %d at (%d, %d, %d), Logged at: %s\n",
                    i + 1, collision_log[i].time_step,
                    collision_log[i].drone_id1, collision_log[i].drone_id2,
                    collision_log[i].x, collision_log[i].y, collision_log[i].z,
                    time_str_buffer);
        }
    }

    fprintf(report_file, "\nOverall Simulation Status: ");
    switch (simulation_status_code) {
        case 0: fprintf(report_file, "PASSED (All drones completed without critical issues)\n"); break;
        case 1: fprintf(report_file, "COMPLETED WITH COLLISIONS\n"); break;
        case 2: fprintf(report_file, "FAILED (Collision threshold exceeded)\n"); break;
        case 3: fprintf(report_file, "FAILED (Not all drones completed their flight plan normally or other critical error)\n"); break;
        default: fprintf(report_file, "UNKNOWN STATUS\n"); break;
    }
    fprintf(report_file, "==========================\n");
    fflush(report_file);
}



// Closes the report file.
void close_report(void) {
    if (report_file) {
        fprintf(report_file, "\n==== End of Report ====\n");
        fclose(report_file);
        report_file = NULL;
    }
}