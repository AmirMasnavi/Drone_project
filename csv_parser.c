// csv_parser.c
#include "drone_simulation.h" // Includes all necessary headers and definitions

// Converts an instruction string (e.g., "UP") to its CommandType enum equivalent.
CommandType string_to_command(const char* str) {
    if (strcmp(str, "UP") == 0) return CMD_UP;
    if (strcmp(str, "DOWN") == 0) return CMD_DOWN;
    if (strcmp(str, "LEFT") == 0) return CMD_LEFT;
    if (strcmp(str, "RIGHT") == 0) return CMD_RIGHT;
    if (strcmp(str, "FORWARD") == 0) return CMD_FORWARD;
    if (strcmp(str, "BACKWARD") == 0) return CMD_BACKWARD;
    if (strcmp(str, "SHAKE") == 0) return CMD_SHAKE;
    if (strcmp(str, "ROTATE") == 0) return CMD_ROTATE;
    fprintf(stderr, "Error: Unknown command string '%s'\n", str);
    return CMD_UNKNOWN;
}

// Converts a CommandType enum back to its string representation for logging.
const char* command_to_string(CommandType cmd) {
    switch (cmd) {
        case CMD_UP: return "UP";
        case CMD_DOWN: return "DOWN";
        case CMD_LEFT: return "LEFT";
        case CMD_RIGHT: return "RIGHT";
        case CMD_FORWARD: return "FORWARD";
        case CMD_BACKWARD: return "BACKWARD";
        case CMD_SHAKE: return "SHAKE";
        case CMD_ROTATE: return "ROTATE";
        default: return "UNKNOWN";
    }
}

// Loads drone configurations from a CSV file.
// Populates the `drones_arr` and updates `drone_count_ptr`.
// Returns 1 on success, 0 on failure.
int load_drones_from_csv(const char* filename, Drone drones_arr[], int* drone_count_ptr) {
    FILE* file = fopen(filename, "r");
    if (!file) {
        perror("CSV_PARSER: Error opening CSV file");
        return 0;
    }

    char line[BUFFER_SIZE];
    *drone_count_ptr = 0;

    // Skip header line
    if (fgets(line, sizeof(line), file) == NULL) {
        fprintf(stderr, "CSV_PARSER: Error reading header or empty file.\n");
        fclose(file);
        return 0;
    }

    while (fgets(line, sizeof(line), file) && *drone_count_ptr < MAX_DRONES) {
        Drone* d = &drones_arr[*drone_count_ptr];
        d->current_instruction_index_tracker = 0;
        d->num_instructions = 0;
        d->active = 1;
        d->pipe_to_child_write_fd = -1;  // Initialize pipe FDs
        d->pipe_from_child_read_fd = -1;

        char* token;

        // Drone ID
        token = strtok(line, ",");
        if (token) d->id = atoi(token); else { fclose(file); return 0; }

        // Start X
        token = strtok(NULL, ",");
        if (token) d->x = atoi(token); else { fclose(file); return 0; }

        // Start Y
        token = strtok(NULL, ",");
        if (token) d->y = atoi(token); else { fclose(file); return 0; }

        // Start Z
        token = strtok(NULL, ",");
        if (token) d->z = atoi(token); else { fclose(file); return 0; }

        // Instructions (semicolon-separated)
        token = strtok(NULL, "\n"); // Read the rest of the line for instructions
        if (token) {
            char* instr_token = strtok(token, ";");
            while (instr_token && d->num_instructions < MAX_INSTRUCTIONS) {
                // Trim leading/trailing whitespace from instruction token
                while (*instr_token == ' ' || *instr_token == '\t') instr_token++;
                char *end = instr_token + strlen(instr_token) - 1;
                while (end > instr_token && (*end == ' ' || *end == '\t')) end--;
                *(end + 1) = '\0'; // Null-terminate the trimmed string

                CommandType cmd = string_to_command(instr_token);
                if (cmd == CMD_UNKNOWN) {
                    fprintf(stderr, "CSV_PARSER: Invalid instruction '%s' for Drone ID %d.\n", instr_token, d->id);
                    fclose(file);
                    return 0; // Critical error if an instruction is unknown
                }
                d->instructions[d->num_instructions++] = cmd;
                instr_token = strtok(NULL, ";");
            }
        }
        (*drone_count_ptr)++;
    }

    fclose(file);
    if (*drone_count_ptr == 0) {
        fprintf(stderr, "CSV_PARSER: No drones loaded from CSV.\n");
        // return 0; // Decide if this is an error or valid (empty simulation)
    }
    return 1;
}