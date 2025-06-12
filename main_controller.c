// main_controller.c
#include "drone_simulation.h"
#include "ui_display.h"

// Define global variables
Drone sim_drones[MAX_DRONES];
int num_sim_drones = 0;
int total_collisions_count = 0; // Mirrored from shared_mem for convenience
int drone_positions_history[MAX_TIME_STEPS][MAX_DRONES][3];
SharedMemoryLayout *shared_mem = NULL;

void cleanup_simulation_resources() {
    // Unlink semaphores
    char sem_name[BUFFER_SIZE];
    for (int i = 0; i < num_sim_drones; ++i) {
        if (sim_drones[i].sem_parent_can_read) sem_close(sim_drones[i].sem_parent_can_read);
        if (sim_drones[i].sem_child_can_act) sem_close(sim_drones[i].sem_child_can_act);

        snprintf(sem_name, BUFFER_SIZE, "%s%d", SEM_PARENT_PREFIX, i);
        sem_unlink(sem_name);
        snprintf(sem_name, BUFFER_SIZE, "%s%d", SEM_CHILD_PREFIX, i);
        sem_unlink(sem_name);
    }

    // Unmap and unlink shared memory
    if (shared_mem) {
        munmap(shared_mem, sizeof(SharedMemoryLayout));
    }
    shm_unlink(SHM_NAME);
    printf("MAIN_CONTROLLER: All simulation resources cleaned up.\n");
}


int main(int argc, char *argv[]) {
    atexit(cleanup_simulation_resources); // Ensure cleanup on normal exit

    const char* csv_filename = "drones_flight_plan.csv";
    if (argc > 1) csv_filename = argv[1];
    printf("MAIN_CONTROLLER: Using drone data file: %s\n", csv_filename);

    init_display();
    if (!init_report(REPORT_FILENAME)) {
        fprintf(stderr, "MAIN_CONTROLLER: CRITICAL - Failed to initialize report file.\n");
        return EXIT_FAILURE;
    }
    log_to_report("MAIN_CONTROLLER: Simulation process started using %s.\n", csv_filename);

    if (!load_drones_from_csv(csv_filename, sim_drones, &num_sim_drones)) {
        fprintf(stderr, "MAIN_CONTROLLER: Failed to load drones from CSV.\n");
        log_error_to_report("Failed to load drone data from CSV.");
        log_simulation_summary_to_report(0, 3);
        close_report();
        return EXIT_FAILURE;
    }

    if (num_sim_drones == 0) {
        printf("MAIN_CONTROLLER: No drones to simulate.\n");
        log_to_report("No drones found in CSV. Simulation ended.\n");
        log_simulation_summary_to_report(0, 0);
        close_report();
        return EXIT_SUCCESS;
    }
    printf("MAIN_CONTROLLER: Loaded %d drones.\n", num_sim_drones);

    // --- Setup Shared Memory ---
    int shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("MAIN_CONTROLLER: shm_open failed");
        return EXIT_FAILURE;
    }
    ftruncate(shm_fd, sizeof(SharedMemoryLayout));
    shared_mem = mmap(0, sizeof(SharedMemoryLayout), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shared_mem == MAP_FAILED) {
        perror("MAIN_CONTROLLER: mmap failed");
        return EXIT_FAILURE;
    }
    close(shm_fd); // File descriptor no longer needed after mmap

    // --- Setup Semaphores and Initialize Shared Memory State ---
    char sem_name[BUFFER_SIZE];
    for (int i = 0; i < num_sim_drones; ++i) {
        // Parent semaphore (parent waits)
        snprintf(sem_name, BUFFER_SIZE, "%s%d", SEM_PARENT_PREFIX, i);
        sem_unlink(sem_name); // Clean up previous runs
        sim_drones[i].sem_parent_can_read = sem_open(sem_name, O_CREAT, 0666, 0); // Initial value 0

        // Child semaphore (child waits)
        snprintf(sem_name, BUFFER_SIZE, "%s%d", SEM_CHILD_PREFIX, i);
        sem_unlink(sem_name); // Clean up previous runs
        sim_drones[i].sem_child_can_act = sem_open(sem_name, O_CREAT, 0666, 0); // Initial value 0

        if (sim_drones[i].sem_parent_can_read == SEM_FAILED || sim_drones[i].sem_child_can_act == SEM_FAILED) {
            perror("MAIN_CONTROLLER: sem_open failed");
            return EXIT_FAILURE;
        }

        // Initialize shared state for this drone
        shared_mem->drones[i].id = sim_drones[i].id;
        shared_mem->drones[i].x = sim_drones[i].initial_x;
        shared_mem->drones[i].y = sim_drones[i].initial_y;
        shared_mem->drones[i].z = sim_drones[i].initial_z;
        shared_mem->drones[i].active = 1;
        shared_mem->drones[i].finished = 0;
        shared_mem->drones[i].terminate_flag = 0;
        drone_positions_history[0][i][0] = sim_drones[i].initial_x;
        drone_positions_history[0][i][1] = sim_drones[i].initial_y;
        drone_positions_history[0][i][2] = sim_drones[i].initial_z;
    }
    shared_mem->total_collisions_count = 0;
    shared_mem->simulation_running = 1;
    
    log_initial_drone_states_to_report();
    printf("Starting simulation...\n");

    // --- Fork Drone Processes ---
    for (int i = 0; i < num_sim_drones; ++i) {
        sim_drones[i].pid = fork();
        if (sim_drones[i].pid < 0) {
            perror("MAIN_CONTROLLER: Error forking drone process");
            shared_mem->simulation_running = 0; // Stop simulation
            return EXIT_FAILURE;
        } else if (sim_drones[i].pid == 0) { // Child Process
            if (report_file) fclose(report_file);
            drone_child_process(i, sim_drones[i]);
            exit(EXIT_SUCCESS);
        } else { // Parent Process
            shared_mem->drones[i].pid = sim_drones[i].pid; // Store PID in shared memory
            log_to_report("MAIN_CONTROLLER: Launched Drone ID %d (PID: %d).\n", sim_drones[i].id, sim_drones[i].pid);
        }
    }

    // --- Main Simulation Loop ---
    int active_drones_count = num_sim_drones;
    int current_time_step = 1;
    int simulation_terminated_by_collisions = 0;
    int overall_simulation_status_code = 0;

    while (active_drones_count > 0 && current_time_step < MAX_TIME_STEPS && !simulation_terminated_by_collisions) {
        log_time_step_header_to_report(current_time_step);
        printf("\n--- Time Step %d ---\n", current_time_step);

        // 1. Signal Phase: Tell active drones to proceed
        for (int i = 0; i < num_sim_drones; ++i) {
            if (shared_mem->drones[i].active) {
                sem_post(sim_drones[i].sem_child_can_act);
            }
        }

        // 2. Wait and Update Phase: Wait for updates from all active drones
        int drones_finished_this_step = 0;
        for (int i = 0; i < num_sim_drones; ++i) {
            if (shared_mem->drones[i].active) {
                sem_wait(sim_drones[i].sem_parent_can_read); // Wait for drone to complete its step
                // Update is already in shared memory, now just log and process it
                if (shared_mem->drones[i].finished) {
                    shared_mem->drones[i].active = 0;
                    drones_finished_this_step++;
                    log_drone_finish_to_report(&shared_mem->drones[i]);
                } else {
                    log_drone_update_to_report(&shared_mem->drones[i], sim_drones[i].instructions[shared_mem->drones[i].instruction_executed_index]);
                }
                // Store position history
                drone_positions_history[current_time_step][i][0] = shared_mem->drones[i].x;
                drone_positions_history[current_time_step][i][1] = shared_mem->drones[i].y;
                drone_positions_history[current_time_step][i][2] = shared_mem->drones[i].z;
            }
        }
        active_drones_count -= drones_finished_this_step;

        display_drone_grid(current_time_step);
        display_drone_summary_list(current_time_step);
        printf("\n");

        // 3. Collision Detection Phase
        log_to_report("Collision checks for this step:\n");
        printf("Collision checks for this step:\n");
        int collisions_this_step_count = 0;
        for (int i = 0; i < num_sim_drones; ++i) {
            // Only check drones that are still in the air or just finished
            if (shared_mem->drones[i].finished && shared_mem->drones[i].instruction_executed_index < current_time_step -1) continue;
            for (int j = i + 1; j < num_sim_drones; ++j) {
                if (shared_mem->drones[j].finished && shared_mem->drones[j].instruction_executed_index < current_time_step -1) continue;

                if (shared_mem->drones[i].x == shared_mem->drones[j].x &&
                    shared_mem->drones[i].y == shared_mem->drones[j].y &&
                    shared_mem->drones[i].z == shared_mem->drones[j].z) {
                    
                    log_collision_to_report(shared_mem->drones[i].id, shared_mem->drones[j].id, shared_mem->drones[i].x, shared_mem->drones[i].y, shared_mem->drones[i].z, current_time_step);
                    printf("  COLLISION! Drone ID %d and Drone ID %d at (%d, %d, %d)\n", shared_mem->drones[i].id, shared_mem->drones[j].id, shared_mem->drones[i].x, shared_mem->drones[i].y, shared_mem->drones[i].z);
                    
                    collisions_this_step_count++;
                    shared_mem->total_collisions_count++;
                    total_collisions_count = shared_mem->total_collisions_count; // sync local copy
                    if (overall_simulation_status_code == 0) overall_simulation_status_code = 1;

                    kill(shared_mem->drones[i].pid, SIGUSR1);
                    kill(shared_mem->drones[j].pid, SIGUSR1);
                }
            }
        }
        if (collisions_this_step_count == 0) {
            log_to_report("  No collisions detected this step.\n");
            printf("  No collisions detected this step.\n");
        }
        printf("---------------------------------\n");

        if (shared_mem->total_collisions_count >= COLLISION_THRESHOLD) {
            printf("\nCRITICAL: Collision threshold (%d) reached. Terminating simulation.\n", COLLISION_THRESHOLD);
            log_to_report("\nCRITICAL: Collision threshold reached. Terminating.\n");
            simulation_terminated_by_collisions = 1;
            overall_simulation_status_code = 2;
        }

        current_time_step++;
    }

    // --- Finalize Simulation ---
    shared_mem->simulation_running = 0; // Signal all children to stop
    for (int i = 0; i < num_sim_drones; ++i) {
        if(shared_mem->drones[i].active) {
            shared_mem->drones[i].terminate_flag = 1;
            sem_post(sim_drones[i].sem_child_can_act); // Unblock any waiting child
        }
    }

    log_simulation_summary_to_report(current_time_step, overall_simulation_status_code);
    close_report();

    printf("\nMAIN_CONTROLLER: Simulation ended. Waiting for child processes...\n");
    for (int i = 0; i < num_sim_drones; ++i) {
        if (sim_drones[i].pid > 0) waitpid(sim_drones[i].pid, NULL, 0);
    }

    printf("MAIN_CONTROLLER: All child processes terminated.\n");
    return (overall_simulation_status_code > 1) ? EXIT_FAILURE : EXIT_SUCCESS;
}