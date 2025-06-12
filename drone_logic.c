// drone_logic.c
#include "drone_simulation.h"

// Signal handler for SIGUSR1 in the child drone process.
// Indicates a collision was detected by the parent.
void signal_handler_collision_child(int signum) {
    if (signum == SIGUSR1) {
        char msg_buff[100];
        int len = snprintf(msg_buff, sizeof(msg_buff),
                           "DRONE_LOGIC (PID %d): Collision signal (SIGUSR1) received. Acknowledged.\n", getpid());
        if (len > 0) {
            write(STDOUT_FILENO, msg_buff, len); // Write directly to stdout for immediate feedback
        }
        // The drone might choose to take specific action here,
        // or just acknowledge and continue if parent handles termination.
    }
}

// Main function executed by each child drone process.
// Manages drone movement based on commands from the parent.
void drone_child_process(Drone initial_drone_config, int pipe_read_from_parent_fd, int pipe_write_to_parent_fd) {
    // Setup signal handler for SIGUSR1 (collision notification)
    struct sigaction sa_collision;
    memset(&sa_collision, 0, sizeof(sa_collision));
    sa_collision.sa_handler = signal_handler_collision_child;
    sigemptyset(&sa_collision.sa_mask);
    // Block other common signals during SIGUSR1 handling to prevent interruption
    sigaddset(&sa_collision.sa_mask, SIGINT);
    sigaddset(&sa_collision.sa_mask, SIGTERM);
    sa_collision.sa_flags = SA_RESTART; // Restart syscalls if interrupted by this signal
    if (sigaction(SIGUSR1, &sa_collision, NULL) == -1) {
        perror("DRONE_LOGIC: Child sigaction failed for SIGUSR1");
        // Not exiting here, as it's a non-critical setup failure for this handler,
        // but logging it is important.
    }

    Drone current_drone_state = initial_drone_config; // Local copy of drone's state
    DroneUpdate update_to_parent_msg;
    ParentToChildMessage control_msg_from_parent;
    int current_instruction_idx = 0;

    while (1) {
        // Wait for a command (PROCEED or TERMINATE) from the parent
        ssize_t bytes_read = read(pipe_read_from_parent_fd, &control_msg_from_parent, sizeof(ParentToChildMessage));

        if (bytes_read <= 0) { // Parent closed pipe or error
            if (bytes_read == 0) {
                // fprintf(stderr, "DRONE_LOGIC (PID %d): Parent closed pipe. Exiting.\n", getpid());
            } else {
                perror("DRONE_LOGIC: Child error reading from parent pipe");
            }
            break; // Exit loop
        }

        if (control_msg_from_parent.type == MSG_TYPE_PROCEED) {
            if (current_instruction_idx < current_drone_state.num_instructions) {
                CommandType cmd = current_drone_state.instructions[current_instruction_idx];

                // Apply the command to update drone's local state
                switch (cmd) {
                    case CMD_UP: current_drone_state.z++; break;
                    case CMD_DOWN: current_drone_state.z--; break;
                    case CMD_LEFT: current_drone_state.x--; break;
                    case CMD_RIGHT: current_drone_state.x++; break;
                    case CMD_FORWARD: current_drone_state.y++; break;
                    case CMD_BACKWARD: current_drone_state.y--; break;
                    case CMD_SHAKE: /* No position change */ break;
                    case CMD_ROTATE: /* No position change */ break;
                    case CMD_UNKNOWN:
                        // fprintf(stderr, "DRONE_LOGIC (PID %d): Executing UNKNOWN command.\n", getpid());
                        break; // Should ideally not happen if CSV parsing is robust
                    default: break;
                }

                // Prepare update message for the parent
                update_to_parent_msg.drone_id = current_drone_state.id;
                update_to_parent_msg.new_x = current_drone_state.x;
                update_to_parent_msg.new_y = current_drone_state.y;
                update_to_parent_msg.new_z = current_drone_state.z;
                update_to_parent_msg.finished = 0; // Not finished yet
                update_to_parent_msg.instruction_executed_index = current_instruction_idx;
                current_instruction_idx++;

            } else { // All instructions have been executed
                update_to_parent_msg.drone_id = current_drone_state.id;
                update_to_parent_msg.new_x = current_drone_state.x; // Send final position
                update_to_parent_msg.new_y = current_drone_state.y;
                update_to_parent_msg.new_z = current_drone_state.z;
                update_to_parent_msg.finished = 1;
                update_to_parent_msg.instruction_executed_index = (current_instruction_idx > 0) ? current_instruction_idx - 1 : 0;
            }

            // Send the update back to the parent
            if (write(pipe_write_to_parent_fd, &update_to_parent_msg, sizeof(DroneUpdate)) == -1) {
                perror("DRONE_LOGIC: Child error writing update to parent pipe");
                break; // Exit loop on write error
            }

            if (update_to_parent_msg.finished) {
                break; // Drone's job is done, exit loop
            }

        } else if (control_msg_from_parent.type == MSG_TYPE_TERMINATE) {
            // char term_msg_buf[100];
            // snprintf(term_msg_buf, sizeof(term_msg_buf),
            //          "DRONE_LOGIC (PID %d, ID %d): Termination message received. Exiting.\n", getpid(), current_drone_state.id);
            // write(STDOUT_FILENO, term_msg_buf, strlen(term_msg_buf)); // For debugging
            break; // Exit loop
        }
    }

    // Clean up: close pipe ends
    close(pipe_read_from_parent_fd);
    close(pipe_write_to_parent_fd);
    exit(EXIT_SUCCESS); // Child process terminates
}