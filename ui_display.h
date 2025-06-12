// ui_display.h
#ifndef UI_DISPLAY_H
#define UI_DISPLAY_H

#include "drone_simulation.h" // For Drone struct and MAX_DRONES

// Define grid dimensions for the text UI
#define GRID_WIDTH 20
#define GRID_HEIGHT 10

// Function to initialize the display (e.g., clear screen if desired)
void init_display(void);

// Function to display the current state of all drones on a text grid
void display_drone_grid(int current_time_step);

// Function to print a summary of drone states (already somewhat in main_controller)
void display_drone_summary_list(int current_time_step);

#endif // UI_DISPLAY_H