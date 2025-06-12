// main_controller.c
#include "drone_simulation.h"
#include "ui_display.h"

// Define global variables (declared extern in drone_simulation.h)
Drone sim_drones[MAX_DRONES];
int num_sim_drones = 0;
int total_collisions_count = 0;
// History of drone positions [time_step_idx][drone_array_idx][x=0,y=1,z=2]
// time_step_idx 0 is initial, time_step_idx 1 is after first move, etc.
int drone_positions_history[MAX_TIME_STEPS][MAX_DRONES][3];


void cleanup_pipes(int drone_index) {
    if (sim_drones[drone_index].pipe_to_child_write_fd != -1) {
        close(sim_drones[drone_index].pipe_to_child_write_fd);
        sim_drones[drone_index].pipe_to_child_write_fd = -1;
    }
    if (sim_drones[drone_index].pipe_from_child_read_fd != -1) {
        close(sim_drones[drone_index].pipe_from_child_read_fd);
        sim_drones[drone_index].pipe_from_child_read_fd = -1;
    }
}

int main(int argc, char *argv[]) {
    const char* csv_filename = "drones_flight_plan.csv";

    if (argc > 1) {
        csv_filename = argv[1]; // Use filename from command line if provided
    }
    printf("MAIN_CONTROLLER: Using drone data file: %s\n", csv_filename);

    init_display();
    if (!init_report(REPORT_FILENAME)) {
        fprintf(stderr, "MAIN_CONTROLLER: CRITICAL - Failed to initialize report file. Exiting.\n");
        return EXIT_FAILURE;
    }

    log_to_report("MAIN_CONTROLLER: Simulation process started using %s.\n", csv_filename);

    if (!load_drones_from_csv(csv_filename, sim_drones, &num_sim_drones)) {
        fprintf(stderr, "MAIN_CONTROLLER: Failed to load drones from CSV.\n");
        log_error_to_report("Failed to load drone data from drones_flight_plan.csv.");
        log_simulation_summary_to_report(0, 3); // Status 3: Failed (Incomplete)
        close_report();
        return EXIT_FAILURE;
    }

    if (num_sim_drones == 0) {
        printf("MAIN_CONTROLLER: No drones to simulate.\n");
        log_to_report("No drones found in CSV. Simulation ended.\n");
        log_simulation_summary_to_report(0, 0); // Status 0: Passed ( vacuously true)
        close_report();
        return EXIT_SUCCESS;
    }

    log_initial_drone_states_to_report();
    printf("MAIN_CONTROLLER: Loaded %d drones. Initial positions logged to report.\n", num_sim_drones);
    printf("Starting simulation...\n");

    // Store initial positions (time_step 0)
    for (int i = 0; i < num_sim_drones; ++i) {
        if (0 < MAX_TIME_STEPS && i < MAX_DRONES) { // Bounds check
            drone_positions_history[0][i][0] = sim_drones[i].x;
            drone_positions_history[0][i][1] = sim_drones[i].y;
            drone_positions_history[0][i][2] = sim_drones[i].z;
        }
    }


    // --- Fork Drone Processes ---
    // Temporary pipe arrays for setup
    int temp_pipes_to_child[MAX_DRONES][2];
    int temp_pipes_from_child[MAX_DRONES][2];

    for (int i = 0; i < num_sim_drones; ++i) {
        if (pipe(temp_pipes_to_child[i]) == -1 || pipe(temp_pipes_from_child[i]) == -1) {
            perror("MAIN_CONTROLLER: Error creating pipes");
            log_error_to_report("Critical error creating pipes for inter-process communication.");
            // Proper cleanup for already opened pipes and forked processes would be needed here
            log_simulation_summary_to_report(0, 3);
            close_report();
            return EXIT_FAILURE;
        }

        sim_drones[i].pid = fork();

        if (sim_drones[i].pid < 0) {
            perror("MAIN_CONTROLLER: Error forking drone process");
            log_error_to_report("Critical error forking a drone child process.");
            // Cleanup
            log_simulation_summary_to_report(0, 3);
            close_report();
            return EXIT_FAILURE;
        } else if (sim_drones[i].pid == 0) { // Child Process
            // Child closes unused ends of its pipes
            close(temp_pipes_to_child[i][1]);    // Does not write to "to_child"
            close(temp_pipes_from_child[i][0]);  // Does not read from "from_child"
            // Report file is managed by parent, child should not touch it after fork
            // unless specifically designed to (not the case here).
            if (report_file) { // Best practice: child closes inherited file descriptors it won't use
                fclose(report_file);
                report_file = NULL;
            }
            drone_child_process(sim_drones[i], temp_pipes_to_child[i][0], temp_pipes_from_child[i][1]);
            exit(EXIT_SUCCESS); // Should be unreachable as drone_child_process calls exit
        } else { // Parent Process
            // Parent closes unused ends of its pipes
            close(temp_pipes_to_child[i][0]);    // Does not read from "to_child"
            close(temp_pipes_from_child[i][1]);  // Does not write to "from_child"

            sim_drones[i].pipe_to_child_write_fd = temp_pipes_to_child[i][1];
            sim_drones[i].pipe_from_child_read_fd = temp_pipes_from_child[i][0];
            log_to_report("MAIN_CONTROLLER: Launched Drone ID %d (PID: %d).\n", sim_drones[i].id, sim_drones[i].pid);
        }
    }

    // --- Main Simulation Loop ---
    int active_drones_count = num_sim_drones;
    int current_time_step = 1; // Start from time step 1 for movements
    int simulation_terminated_by_collisions = 0;
    int overall_simulation_status_code = 0; // 0=OK, 1=Collisions, 2=Threshold, 3=Error/Incomplete


    while (active_drones_count > 0 && current_time_step < MAX_TIME_STEPS) {
        log_time_step_header_to_report(current_time_step);
        printf("\n--- Time Step %d ---\n", current_time_step);

        ParentToChildMessage proceed_msg = {MSG_TYPE_PROCEED};
        ParentToChildMessage terminate_msg = {MSG_TYPE_TERMINATE}; // For early termination

        // Check if simulation needs to terminate due to excessive collisions
        if (simulation_terminated_by_collisions) {
            log_to_report("MAIN_CONTROLLER: Terminating remaining active drones due to collision threshold.\n");
            printf("MAIN_CONTROLLER: Terminating remaining active drones due to collision threshold.\n");
            for (int i = 0; i < num_sim_drones; ++i) {
                if (sim_drones[i].active) {
                    if (sim_drones[i].pipe_to_child_write_fd != -1) { // Check if pipe is open
                         if (write(sim_drones[i].pipe_to_child_write_fd, &terminate_msg, sizeof(ParentToChildMessage)) == -1) {
                            //perror("MAIN_CONTROLLER: Error sending TERMINATE message");
                            // Log this minor error if needed, but proceed with cleanup
                         }
                    }
                    cleanup_pipes(i);
                    sim_drones[i].active = 0;
                }
            }
            active_drones_count = 0; // All drones are now considered inactive
            overall_simulation_status_code = 2; // Failed due to threshold
            continue; // Skip to end of while loop
        }

        // 1. Signal Phase: Tell active drones to proceed
        for (int i = 0; i < num_sim_drones; ++i) {
            if (sim_drones[i].active) {
                if (write(sim_drones[i].pipe_to_child_write_fd, &proceed_msg, sizeof(ParentToChildMessage)) == -1) {
                    perror("MAIN_CONTROLLER: Error writing PROCEED to child");
                    log_error_to_report("Error sending PROCEED command to drone. Marking inactive.");
                    cleanup_pipes(i);
                    sim_drones[i].active = 0;
                    active_drones_count--;
                    overall_simulation_status_code = 3; // Mark as incomplete/error
                }
            }
        }

        // 2. Update Phase: Collect updates from active drones
        log_to_report("Drone states after this step's movements:\n");
        // printf("Drone states after this step's movements:\n");
        display_drone_grid(current_time_step);
        display_drone_summary_list(current_time_step);
        printf("\n"); 
        for (int i = 0; i < num_sim_drones; ++i) {
            if (sim_drones[i].active) {
                DroneUpdate received_update;
                ssize_t bytes_read = read(sim_drones[i].pipe_from_child_read_fd, &received_update, sizeof(DroneUpdate));

                if (bytes_read > 0) {
                    sim_drones[i].x = received_update.new_x;
                    sim_drones[i].y = received_update.new_y;
                    sim_drones[i].z = received_update.new_z;
                    sim_drones[i].current_instruction_index_tracker++;

                    // Store position history
                    if (current_time_step < MAX_TIME_STEPS && i < MAX_DRONES) {
                         drone_positions_history[current_time_step][i][0] = sim_drones[i].x;
                         drone_positions_history[current_time_step][i][1] = sim_drones[i].y;
                         drone_positions_history[current_time_step][i][2] = sim_drones[i].z;
                    }


                    if (received_update.finished) {
                        sim_drones[i].active = 0;
                        active_drones_count--;
                        log_drone_finish_to_report(&sim_drones[i]);
                        // printf("  Drone ID %d: (%d, %d, %d) - FINISHED\n", sim_drones[i].id, sim_drones[i].x, sim_drones[i].y, sim_drones[i].z);
                        cleanup_pipes(i);
                    } else {
                        log_drone_update_to_report(&sim_drones[i], &received_update, sim_drones[i].instructions[received_update.instruction_executed_index]);
                        // printf("  Drone ID %d: (%d, %d, %d) - Active (Executed instr %d: %s)\n",
                        //        sim_drones[i].id, sim_drones[i].x, sim_drones[i].y, sim_drones[i].z,
                        //        received_update.instruction_executed_index,
                        //        command_to_string(sim_drones[i].instructions[received_update.instruction_executed_index]));
                    }
                } else { // Error reading or pipe closed prematurely
                    fprintf(stderr, "MAIN_CONTROLLER: Error or premature pipe close from Drone ID %d.\n", sim_drones[i].id);
                    log_error_to_report("Error reading update from drone or pipe closed prematurely. Marking inactive.");
                    cleanup_pipes(i);
                    sim_drones[i].active = 0;
                    active_drones_count--;
                    overall_simulation_status_code = 3;
                }
            }
        }

        // 3. Collision Detection Phase
        log_to_report("Collision checks for this step:\n");
        printf("Collision checks for this step:\n");
        int collisions_this_step_count = 0;
        if (!simulation_terminated_by_collisions) { // Only check if not already terminating
            for (int i = 0; i < num_sim_drones; ++i) {
                // Check drone 'i' if it was active before this step's update OR just finished in this step
                if (!sim_drones[i].active && sim_drones[i].current_instruction_index_tracker <= current_time_step) continue;

                for (int j = i + 1; j < num_sim_drones; ++j) {
                    if (!sim_drones[j].active && sim_drones[j].current_instruction_index_tracker <= current_time_step) continue;

                    if (sim_drones[i].x == sim_drones[j].x &&
                        sim_drones[i].y == sim_drones[j].y &&
                        sim_drones[i].z == sim_drones[j].z)
                    {
                        // A collision occurred if both drones are at the same spot AND
                        // at least one of them was active enough to have potentially moved there this step,
                        // or finished there this step. The current_instruction_index_tracker check helps.
                        // If a drone is !active but its tracker is >= current_time_step, it means it just finished.
                        int drone_i_participated = sim_drones[i].active || (sim_drones[i].current_instruction_index_tracker >= current_time_step);
                        int drone_j_participated = sim_drones[j].active || (sim_drones[j].current_instruction_index_tracker >= current_time_step);

                        if (drone_i_participated && drone_j_participated) {
                            log_collision_to_report(sim_drones[i].id, sim_drones[j].id, sim_drones[i].x, sim_drones[i].y, sim_drones[i].z, current_time_step);
                            printf("  COLLISION! Drone ID %d and Drone ID %d at (%d, %d, %d)\n",
                                   sim_drones[i].id, sim_drones[j].id, sim_drones[i].x, sim_drones[i].y, sim_drones[i].z);

                            collisions_this_step_count++;
                            total_collisions_count++;
                            if (overall_simulation_status_code == 0) overall_simulation_status_code = 1; // Mark as completed with collisions

                            // Send SIGUSR1 to involved drones
                            if (sim_drones[i].pid > 0) kill(sim_drones[i].pid, SIGUSR1); else log_to_report("Note: Drone %d PID invalid for SIGUSR1.\n", sim_drones[i].id);
                            if (sim_drones[j].pid > 0) kill(sim_drones[j].pid, SIGUSR1); else log_to_report("Note: Drone %d PID invalid for SIGUSR1.\n", sim_drones[j].id);

                            // Optional: Deactivate drones on collision
                            // sim_drones[i].active = 0; active_drones_count--; cleanup_pipes(i);
                            // sim_drones[j].active = 0; active_drones_count--; cleanup_pipes(j);
                        }
                    }
                }
            }
        }
        if (collisions_this_step_count == 0 && !simulation_terminated_by_collisions) {
            log_to_report("  No collisions detected this step.\n");
            printf("  No collisions detected this step.\n");
        }
        log_to_report("---------------------------------\n");
        printf("---------------------------------\n");


        // Check collision threshold
        if (total_collisions_count >= COLLISION_THRESHOLD && !simulation_terminated_by_collisions) {
            log_to_report("\nCRITICAL: Collision threshold (%d) reached/exceeded (%d collisions). Initiating termination.\n",
                          COLLISION_THRESHOLD, total_collisions_count);
            printf("\nCRITICAL: Collision threshold (%d) reached/exceeded (%d collisions). Initiating termination.\n",
                   COLLISION_THRESHOLD, total_collisions_count);
            simulation_terminated_by_collisions = 1; // Will trigger termination at start of next loop
            overall_simulation_status_code = 2;
        }

        if (active_drones_count == 0 && !simulation_terminated_by_collisions && overall_simulation_status_code == 0) {
            log_to_report("\nAll drones have successfully completed their flight plans without critical issues.\n");
            printf("\nAll drones have successfully completed their flight plans without critical issues.\n");
        }
        current_time_step++;
    } // End of main simulation loop

    // --- Finalize Simulation ---
    if (current_time_step >= MAX_TIME_STEPS && active_drones_count > 0){
        log_error_to_report("Simulation stopped: Maximum time steps reached.");
        printf("MAIN_CONTROLLER: Simulation stopped: Maximum time steps reached.\n");
        if (overall_simulation_status_code < 2) overall_simulation_status_code = 3; // Incomplete
         // Terminate any remaining active children
        for (int i = 0; i < num_sim_drones; ++i) {
            if (sim_drones[i].active) {
                ParentToChildMessage terminate_msg = {MSG_TYPE_TERMINATE};
                if (sim_drones[i].pipe_to_child_write_fd != -1) {
                    write(sim_drones[i].pipe_to_child_write_fd, &terminate_msg, sizeof(ParentToChildMessage));
                }
                cleanup_pipes(i);
            }
        }
    }


    log_simulation_summary_to_report(current_time_step, overall_simulation_status_code);
    close_report();

    printf("\nMAIN_CONTROLLER: Simulation ended. Waiting for child drone processes to terminate...\n");
    for (int i = 0; i < num_sim_drones; ++i) {
        if (sim_drones[i].pid > 0) { // Check if child was actually forked
            int status;
            waitpid(sim_drones[i].pid, &status, 0); // Wait for child to exit
            // printf("MAIN_CONTROLLER: Drone ID %d (PID %d) terminated with status %d.\n", sim_drones[i].id, sim_drones[i].pid, status);
        }
    }

    printf("MAIN_CONTROLLER: All child processes terminated. Exiting.\n");
    return (overall_simulation_status_code > 1) ? EXIT_FAILURE : EXIT_SUCCESS; // Exit with error if simulation failed critically
}