// ui_display.c
#include "ui_display.h"
#include <stdio.h> // For printf

// --- Externs for accessing simulation state ---
// `sim_drones` holds the initial configuration (e.g., total instructions).
extern Drone sim_drones[MAX_DRONES];
// `shared_mem` holds the LIVE, dynamic state (e.g., current position, status).
extern SharedMemoryLayout *shared_mem;
// `num_sim_drones` is the total count.
extern int num_sim_drones;

void init_display(void) {
    // This function does not depend on drone state, so no changes are needed.
    printf("Initializing Drone Simulation Display...\n");
}

void display_drone_grid(int current_time_step) {
    // Check if shared memory is available. If not, we can't display anything.
    if (!shared_mem) {
        printf("UI_DISPLAY: Shared memory not available. Cannot display grid.\n");
        return;
    }

    char grid[GRID_HEIGHT][GRID_WIDTH];
    int drone_ids_in_cell[GRID_HEIGHT][GRID_WIDTH];

    // Initialize grid with empty spaces
    for (int y = 0; y < GRID_HEIGHT; ++y) {
        for (int x = 0; x < GRID_WIDTH; ++x) {
            grid[y][x] = '.';
            drone_ids_in_cell[y][x] = 0;
        }
    }

    // Place drones on the grid using their LIVE positions from shared memory
    for (int i = 0; i < num_sim_drones; ++i) {
        // We display any drone that hasn't been terminated or is still running.
        // Reading its current position from the shared state.
        int drone_x = shared_mem->drones[i].x;
        int drone_y = shared_mem->drones[i].y;
        int drone_id = shared_mem->drones[i].id;

        // Check if drone is within grid bounds for display
        if (drone_x >= 0 && drone_x < GRID_WIDTH && drone_y >= 0 && drone_y < GRID_HEIGHT) {
            if (grid[drone_y][drone_x] == '.') {
                grid[drone_y][drone_x] = (drone_id % 10) + '0'; // Display drone ID (last digit)
                drone_ids_in_cell[drone_y][drone_x] = drone_id;
            } else {
                grid[drone_y][drone_x] = '*'; // Multiple drones in the same X,Y cell
            }
        }
    }

    // Print the grid (this logic remains the same)
    printf("Time Step %d - Drone Grid (X:0-%d, Y:0-%d):\n", current_time_step, GRID_WIDTH - 1, GRID_HEIGHT - 1);
    printf("  +");
    for (int x = 0; x < GRID_WIDTH; ++x) printf("-");
    printf("+\n");

    for (int y = GRID_HEIGHT - 1; y >= 0; --y) {
        printf("%2d|", y);
        for (int x = 0; x < GRID_WIDTH; ++x) {
            printf("%c", grid[y][x]);
        }
        printf("|\n");
    }
    printf("  +");
    for (int x = 0; x < GRID_WIDTH; ++x) printf("-");
    printf("+\n");
    printf("  Legend: '.' = empty, '0-9' = Drone ID, '*' = multiple drones in cell (X,Y)\n");
}

void display_drone_summary_list(int current_time_step) {
    if (!shared_mem) {
        printf("UI_DISPLAY: Shared memory not available. Cannot display summary.\n");
        return;
    }
    
    printf("Drone States List (Time Step %d):\n", current_time_step);
    for (int i = 0; i < num_sim_drones; ++i) {
        const char* status_str;
        // Determine status string based on LIVE flags from shared memory
        if (shared_mem->drones[i].finished) {
            status_str = "FINISHED";
        } else if (shared_mem->drones[i].active) {
            status_str = "Active";
        } else {
            // This would happen if a drone becomes inactive before finishing (e.g., early termination)
            status_str = "Inactive";
        }

        // Print a summary using data from both shared_mem (live) and sim_drones (config)
        printf("  Drone ID %2d: Pos (%3d, %3d, %3d) - Status: %-10s - Instr: %d/%d\n",
               shared_mem->drones[i].id,
               shared_mem->drones[i].x, shared_mem->drones[i].y, shared_mem->drones[i].z, // Live position
               status_str,                                                                // Live status
               shared_mem->drones[i].instruction_executed_index + 1,                      // Live progress
               sim_drones[i].num_instructions);                                           // Total instructions from config
    }
}