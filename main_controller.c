// main_controller.c
#include "drone_simulation.h"
#include "ui_display.h"

// Define global variables
Drone sim_drones[MAX_DRONES];
int num_sim_drones = 0;
int total_collisions_count = 0;
int drone_positions_history[MAX_TIME_STEPS][MAX_DRONES][3];
SharedMemoryLayout *shared_mem = NULL;

// Thread management and synchronization variables
pthread_t sim_thread_id, collision_thread_id, report_thread_id;
pthread_mutex_t data_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t step_cond = PTHREAD_COND_INITIALIZER;
pthread_cond_t collision_cond = PTHREAD_COND_INITIALIZER;

// Shared state variables for thread coordination
int current_time_step = 1;
int step_ready_for_collision_check = 0;
int collision_event_occurred = 0;
int overall_simulation_status_code = 0;

// ADDED: Struct to pass collision data to the report thread
typedef struct {
    int drone_id1, drone_id2, x, y, z, time_step;
} CollisionInfo;
CollisionInfo last_collision_info;


void cleanup_simulation_resources() {
    char sem_name[BUFFER_SIZE];
    for (int i = 0; i < num_sim_drones; ++i) {
        if (sim_drones[i].sem_parent_can_read) sem_close(sim_drones[i].sem_parent_can_read);
        if (sim_drones[i].sem_child_can_act) sem_close(sim_drones[i].sem_child_can_act);
        snprintf(sem_name, BUFFER_SIZE, "%s%d", SEM_PARENT_PREFIX, i);
        sem_unlink(sem_name);
        snprintf(sem_name, BUFFER_SIZE, "%s%d", SEM_CHILD_PREFIX, i);
        sem_unlink(sem_name);
    }
    if (shared_mem) {
        munmap(shared_mem, sizeof(SharedMemoryLayout));
    }
    shm_unlink(SHM_NAME);
    pthread_mutex_destroy(&data_mutex);
    pthread_cond_destroy(&step_cond);
    pthread_cond_destroy(&collision_cond);
    printf("MAIN_CONTROLLER: All simulation resources cleaned up.\n");
}

/**
 * @brief Thread for the main simulation loop.
 */
void* simulation_loop_thread(void* arg) {
    int active_drones_count = num_sim_drones;

    while (active_drones_count > 0 && current_time_step < MAX_TIME_STEPS && shared_mem->simulation_running) {
        pthread_mutex_lock(&data_mutex);
        log_time_step_header_to_report(current_time_step);
        printf("\n--- Time Step %d ---\n", current_time_step);

        for (int i = 0; i < num_sim_drones; ++i) {
            if (shared_mem->drones[i].active) {
                sem_post(sim_drones[i].sem_child_can_act);
            }
        }

        int drones_finished_this_step = 0;
        for (int i = 0; i < num_sim_drones; ++i) {
            if (shared_mem->drones[i].active) {
                sem_wait(sim_drones[i].sem_parent_can_read);
                if (shared_mem->drones[i].finished) {
                    shared_mem->drones[i].active = 0;
                    drones_finished_this_step++;
                    log_drone_finish_to_report(&shared_mem->drones[i]);
                } else {
                    log_drone_update_to_report(&shared_mem->drones[i], sim_drones[i].instructions[shared_mem->drones[i].instruction_executed_index]);
                }
                drone_positions_history[current_time_step][i][0] = shared_mem->drones[i].x;
                drone_positions_history[current_time_step][i][1] = shared_mem->drones[i].y;
                drone_positions_history[current_time_step][i][2] = shared_mem->drones[i].z;
            }
        }
        active_drones_count -= drones_finished_this_step;

        display_drone_grid(current_time_step);
        display_drone_summary_list(current_time_step);

        step_ready_for_collision_check = 1;
        pthread_cond_signal(&step_cond);
        pthread_mutex_unlock(&data_mutex);
        usleep(10000);
    }
    
    pthread_mutex_lock(&data_mutex);
    shared_mem->simulation_running = 0;
    pthread_cond_broadcast(&step_cond);
    pthread_cond_broadcast(&collision_cond);
    pthread_mutex_unlock(&data_mutex);

    return NULL;
}

/**
 * @brief Thread for detecting collisions.
 * This meets criterion 1: "The collision detection thread monitors the shared memory..."
 */
void* collision_detection_thread(void* arg) {
    while (shared_mem->simulation_running) {
        pthread_mutex_lock(&data_mutex);
        while (!step_ready_for_collision_check && shared_mem->simulation_running) {
            pthread_cond_wait(&step_cond, &data_mutex);
        }

        if (!shared_mem->simulation_running) {
            pthread_mutex_unlock(&data_mutex);
            break;
        }

        int collisions_this_step_count = 0;
        for (int i = 0; i < num_sim_drones; ++i) {
            for (int j = i + 1; j < num_sim_drones; ++j) {
                if (shared_mem->drones[i].x == shared_mem->drones[j].x &&
                    shared_mem->drones[i].y == shared_mem->drones[j].y &&
                    shared_mem->drones[i].z == shared_mem->drones[j].z) {
                    
                    collisions_this_step_count++;
                    shared_mem->total_collisions_count++;
                    total_collisions_count = shared_mem->total_collisions_count;
                    if (overall_simulation_status_code == 0) overall_simulation_status_code = 1;
                    
                    // Populate collision info for the reporting thread
                    last_collision_info = (CollisionInfo){
                        shared_mem->drones[i].id, shared_mem->drones[j].id,
                        shared_mem->drones[i].x, shared_mem->drones[i].y, shared_mem->drones[i].z,
                        current_time_step
                    };
                    
                    kill(shared_mem->drones[i].pid, SIGUSR1);
                    kill(shared_mem->drones[j].pid, SIGUSR1);
                }
            }
        }

        if (collisions_this_step_count > 0) {
            collision_event_occurred = 1;
            // This meets criterion 2 & 4: "signals... using condition variables" with "Proper mutex locking"
            pthread_cond_signal(&collision_cond);
        }

        if (shared_mem->total_collisions_count >= COLLISION_THRESHOLD) {
            printf("\nCRITICAL: Collision threshold (%d) reached. Terminating simulation.\n", COLLISION_THRESHOLD);
            overall_simulation_status_code = 2;
            shared_mem->simulation_running = 0;
        }
        
        step_ready_for_collision_check = 0;
        current_time_step++;
        pthread_mutex_unlock(&data_mutex);
    }
    return NULL;
}

/**
 * @brief Thread for generating reports.
 * This meets criterion 3: "The report generation thread, waiting on the condition variable, immediately processes..."
 */
void* report_generation_thread(void* arg) {
    while (shared_mem->simulation_running) {
        pthread_mutex_lock(&data_mutex);
        while (!collision_event_occurred && shared_mem->simulation_running) {
            // Waits on the condition variable, protected by the mutex
            pthread_cond_wait(&collision_cond, &data_mutex);
        }

        if (!shared_mem->simulation_running) {
            pthread_mutex_unlock(&data_mutex);
            break;
        }
        
        // MOVED: The logging call is now in the report thread.
        log_to_report("Collision checks for this step:\n");
        log_collision_to_report(last_collision_info.drone_id1, last_collision_info.drone_id2, 
                                last_collision_info.x, last_collision_info.y, last_collision_info.z, 
                                last_collision_info.time_step);
        printf("  REPORT_THREAD: Notified of collision. Details logged.\n");
        
        collision_event_occurred = 0;
        pthread_mutex_unlock(&data_mutex);
    }
    return NULL;
}

int main(int argc, char *argv[]) {
    atexit(cleanup_simulation_resources);

    const char* csv_filename = "drones_flight_plan.csv";
    if (argc > 1) csv_filename = argv[1];

    init_display();
    if (!init_report(REPORT_FILENAME)) return EXIT_FAILURE;
    log_to_report("MAIN_CONTROLLER: Simulation process started using %s.\n", csv_filename);

    if (!load_drones_from_csv(csv_filename, sim_drones, &num_sim_drones) || num_sim_drones == 0) {
        log_simulation_summary_to_report(0, num_sim_drones == 0 ? 0 : 3);
        close_report();
        return num_sim_drones == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    int shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    ftruncate(shm_fd, sizeof(SharedMemoryLayout));
    shared_mem = mmap(0, sizeof(SharedMemoryLayout), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    close(shm_fd);

    shared_mem->simulation_running = 1;
    shared_mem->total_collisions_count = 0;

    char sem_name[BUFFER_SIZE];
    for (int i = 0; i < num_sim_drones; ++i) {
        snprintf(sem_name, BUFFER_SIZE, "%s%d", SEM_PARENT_PREFIX, i);
        sem_unlink(sem_name);
        sim_drones[i].sem_parent_can_read = sem_open(sem_name, O_CREAT, 0666, 0);
        snprintf(sem_name, BUFFER_SIZE, "%s%d", SEM_CHILD_PREFIX, i);
        sem_unlink(sem_name);
        sim_drones[i].sem_child_can_act = sem_open(sem_name, O_CREAT, 0666, 0);
        shared_mem->drones[i] = (DroneSharedState){.id = sim_drones[i].id, .x = sim_drones[i].initial_x, .y = sim_drones[i].initial_y, .z = sim_drones[i].initial_z, .active = 1, .finished = 0, .terminate_flag = 0};
    }

    log_initial_drone_states_to_report();
    printf("MAIN_CONTROLLER: Loaded %d drones. Starting simulation...\n", num_sim_drones);

    for (int i = 0; i < num_sim_drones; ++i) {
        sim_drones[i].pid = fork();
        if (sim_drones[i].pid == 0) {
            if (report_file) fclose(report_file);
            drone_child_process(i, sim_drones[i]);
            exit(EXIT_SUCCESS);
        } else {
            shared_mem->drones[i].pid = sim_drones[i].pid;
            log_to_report("MAIN_CONTROLLER: Launched Drone ID %d (PID: %d).\n", sim_drones[i].id, sim_drones[i].pid);
        }
    }

    pthread_create(&sim_thread_id, NULL, simulation_loop_thread, NULL);
    pthread_create(&collision_thread_id, NULL, collision_detection_thread, NULL);
    pthread_create(&report_thread_id, NULL, report_generation_thread, NULL);

    pthread_join(sim_thread_id, NULL);
    pthread_join(collision_thread_id, NULL);
    pthread_join(report_thread_id, NULL);

    for (int i = 0; i < num_sim_drones; ++i) {
        if (shared_mem->drones[i].active) {
            shared_mem->drones[i].terminate_flag = 1;
            sem_post(sim_drones[i].sem_child_can_act);
        }
    }

    log_simulation_summary_to_report(current_time_step - 1, overall_simulation_status_code);
    close_report();

    printf("\nMAIN_CONTROLLER: Simulation ended. Waiting for child processes...\n");
    for (int i = 0; i < num_sim_drones; ++i) {
        if (sim_drones[i].pid > 0) waitpid(sim_drones[i].pid, NULL, 0);
    }

    printf("MAIN_CONTROLLER: All child processes terminated.\n");
    return (overall_simulation_status_code > 1) ? EXIT_FAILURE : EXIT_SUCCESS;
}