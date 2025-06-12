// drone_logic.c
#include "drone_simulation.h"

// Defined globally for access by signal handler and main logic
static volatile sig_atomic_t collision_signal_received = 0;

void signal_handler_collision_child(int signum) {
    if (signum == SIGUSR1) {
        collision_signal_received = 1;
    }
}

void drone_child_process(int drone_index, Drone initial_drone_config) {
    // --- Attach to Shared Memory and Semaphores ---
    SharedMemoryLayout *local_shared_mem;
    int shm_fd = shm_open(SHM_NAME, O_RDWR, 0666);
    if (shm_fd == -1) { perror("DRONE_LOGIC: shm_open child"); exit(EXIT_FAILURE); }
    local_shared_mem = mmap(0, sizeof(SharedMemoryLayout), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (local_shared_mem == MAP_FAILED) { perror("DRONE_LOGIC: mmap child"); exit(EXIT_FAILURE); }
    close(shm_fd);

    char sem_name[BUFFER_SIZE];
    snprintf(sem_name, BUFFER_SIZE, "%s%d", SEM_PARENT_PREFIX, drone_index);
    sem_t *sem_parent = sem_open(sem_name, 0);
    snprintf(sem_name, BUFFER_SIZE, "%s%d", SEM_CHILD_PREFIX, drone_index);
    sem_t *sem_child = sem_open(sem_name, 0);
    if (sem_parent == SEM_FAILED || sem_child == SEM_FAILED) {
        perror("DRONE_LOGIC: sem_open child");
        exit(EXIT_FAILURE);
    }
    
    // Setup signal handler for SIGUSR1
    signal(SIGUSR1, signal_handler_collision_child);

    int current_instruction_idx = 0;
    int drone_id = initial_drone_config.id;
    int my_x = initial_drone_config.initial_x;
    int my_y = initial_drone_config.initial_y;
    int my_z = initial_drone_config.initial_z;

    while (local_shared_mem->simulation_running) {
        sem_wait(sem_child); // Wait for parent to signal "go"

        if (local_shared_mem->drones[drone_index].terminate_flag) {
            break; // Exit if parent ordered termination
        }
        
        if (collision_signal_received) {
             char msg_buff[100];
             snprintf(msg_buff, sizeof(msg_buff), "DRONE_LOGIC (ID %d): Acknowledged collision signal.\n", drone_id);
             write(STDOUT_FILENO, msg_buff, strlen(msg_buff));
             collision_signal_received = 0; // Reset flag
        }

        if (current_instruction_idx < initial_drone_config.num_instructions) {
            CommandType cmd = initial_drone_config.instructions[current_instruction_idx];

            switch (cmd) {
                case CMD_UP: my_z++; break;
                case CMD_DOWN: my_z--; break;
                case CMD_LEFT: my_x--; break;
                case CMD_RIGHT: my_x++; break;
                case CMD_FORWARD: my_y++; break;
                case CMD_BACKWARD: my_y--; break;
                default: break;
            }

            // --- Update shared memory ---
            local_shared_mem->drones[drone_index].x = my_x;
            local_shared_mem->drones[drone_index].y = my_y;
            local_shared_mem->drones[drone_index].z = my_z;
            local_shared_mem->drones[drone_index].instruction_executed_index = current_instruction_idx;
            
            current_instruction_idx++;
            
            if (current_instruction_idx >= initial_drone_config.num_instructions) {
                 local_shared_mem->drones[drone_index].finished = 1;
            }

        } else {
             local_shared_mem->drones[drone_index].finished = 1;
        }

        sem_post(sem_parent); // Signal parent: "I'm done with this step"

        if (local_shared_mem->drones[drone_index].finished) {
            break; // My job is done
        }
    }

    // --- Cleanup ---
    sem_close(sem_parent);
    sem_close(sem_child);
    munmap(local_shared_mem, sizeof(SharedMemoryLayout));
    exit(EXIT_SUCCESS);
}