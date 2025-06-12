// ui_display.c
#include "ui_display.h"
#include <stdio.h> // For printf

// External from drone_simulation.h (defined in main_controller.c)
extern Drone sim_drones[MAX_DRONES];
extern int num_sim_drones;

void init_display(void) {
    // Optional: Clear screen (OS-dependent)
    // printf("\033[H\033[J"); // ANSI escape code for clear screen (Unix-like)
    // system("cls"); // For Windows (less portable)
    printf("Initializing Drone Simulation Display...\n");
}

void display_drone_grid(int current_time_step) {
    char grid[GRID_HEIGHT][GRID_WIDTH];
    int drone_ids_in_cell[GRID_HEIGHT][GRID_WIDTH]; // To store one drone ID, or use count

    // Initialize grid with empty spaces and no drones
    for (int y = 0; y < GRID_HEIGHT; ++y) {
        for (int x = 0; x < GRID_WIDTH; ++x) {
            grid[y][x] = '.';
            drone_ids_in_cell[y][x] = 0; // 0 means no drone, or use -1 if drone IDs can be 0
        }
    }

    // Place drones on the grid
    for (int i = 0; i < num_sim_drones; ++i) {
        if (sim_drones[i].active || sim_drones[i].current_instruction_index_tracker >= current_time_step) { // Drone is active or just finished
            int drone_x = sim_drones[i].x;
            int drone_y = sim_drones[i].y;

            // Check if drone is within grid bounds for display
            if (drone_x >= 0 && drone_x < GRID_WIDTH && drone_y >= 0 && drone_y < GRID_HEIGHT) {
                if (grid[drone_y][drone_x] == '.') {
                    grid[drone_y][drone_x] = (sim_drones[i].id % 10) + '0'; // Display drone ID (last digit)
                    drone_ids_in_cell[drone_y][drone_x] = sim_drones[i].id;
                } else {
                    grid[drone_y][drone_x] = '*'; // Multiple drones in the same X,Y cell
                }
            }
        }
    }

    // Print the grid
    printf("Time Step %d - Drone Grid (X:0-%d, Y:0-%d):\n", current_time_step, GRID_WIDTH-1, GRID_HEIGHT-1);
    printf("  +");
    for (int x = 0; x < GRID_WIDTH; ++x) printf("-");
    printf("+\n");

    for (int y = GRID_HEIGHT - 1; y >= 0; --y) { // Print Y from top to bottom (typical screen coordinates)
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
    printf("Drone States List (Time Step %d):\n", current_time_step);
    for (int i = 0; i < num_sim_drones; ++i) {
        const char* status_str;
        if (!sim_drones[i].active) {
            status_str = "FINISHED";
        } else if (sim_drones[i].current_instruction_index_tracker < sim_drones[i].num_instructions) {
            status_str = "Active";
        } else {
            status_str = "Awaiting Finish Signal"; // Should become FINISHED in parent logic
        }

        printf("  Drone ID %2d: Pos (%3d, %3d, %3d) - Status: %-22s - Next Instr Idx: %d/%d\n",
               sim_drones[i].id,
               sim_drones[i].x, sim_drones[i].y, sim_drones[i].z,
               status_str,
               sim_drones[i].current_instruction_index_tracker,
               sim_drones[i].num_instructions);
    }
}