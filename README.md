# Sprint 3 - Drone Simulation Project

**Group Members:**

* Amir Masnavi - Nº1221579
* Leonor Marinho - Nº1230977
* Inês Oliveira - Nº1231205
* Rui Moreira - Nº1221696

## 1. Solution Component Diagram

![Component-Diagram-of-the-Implemented-Solution.svg](Component-Diagram-of-the-Implemented-Solution.svg)

**Diagram Description:**

The drone simulation system is composed of the following key components and interactions:

* **Main Controller Process (`drone_simulator` executable from `main_controller.c`):**

  * Acts as the central orchestrator.
  * **Reads:** `drones_flight_plan.csv` (via `csv_parser.c`) to get drone initial states and instructions.
  * **Creates:** Shared memory segment (`shm_open`, `mmap`) of type `SharedMemoryLayout`, and POSIX semaphores (`sem_open`) for each drone.
  * **Spawns:** A child process per drone via `fork()`, and three parent threads via `pthread_create`:

    1. **Simulation Loop Thread** (drives time steps)
    2. **Collision Detection Thread**
    3. **Report Generation Thread**
  * **Coordinates:**

    * Uses semaphores (`sem_child_can_act` and `sem_parent_can_read`) to gate each drone’s per-step action and acknowledgment, enforcing lockstep progression.
    * Uses mutex (`data_mutex`) and condition variables (`step_cond`, `collision_cond`) to sequence between simulation, collision detection, and reporting.
  * **Detects:** Collisions by comparing drone positions in shared memory after each step.
  * **Signals:** Sends `SIGUSR1` to child processes of drones involved in collisions.
  * **Writes:** `simulation_report.txt` (via `reporting.c`) with detailed logs, per-step updates, collisions, final statuses, and summary.

* **Drone Child Processes (from `drone_logic.c`, one per drone):**

  * **Attach:** to the shared memory segment and open the two semaphores created for it.
  * **Receives:** A semaphore-post from parent (`sem_child_can_act`) to proceed one instruction per time step.
  * **Executes:** One movement instruction, updates its local state, writes into shared memory (`drones[i]` fields).
  * **Acknowledges:** Parent via `sem_post(sem_parent)` once update is done.
  * **Handles Signals:** Catches `SIGUSR1` (collision notification) through a handler that logs an acknowledgment.
  * **Terminates:** Upon finishing all instructions or receiving a terminate flag in shared memory.

* **Shared Memory (`/drone_sim_shm`):**

  * Stores global simulation flags (`simulation_running`, `total_collisions_count`) and an array of `DroneSharedState` structs, one per drone.

* **Semaphores:**

  * Two POSIX semaphores per drone named `/sim_parent_sem_i` and `/sim_child_sem_i` for controlling step progression.

* **Files:**

  * `drones_flight_plan.csv` (Input): Initial positions and semicolon-separated commands.
  * `simulation_report.txt` (Output): Logs of initial states, per-step updates, collisions, final statuses, and summary.

---

## 2. Example Drone Movement Script

The movement scripts for drones are defined in the input CSV file (e.g., `drones_flight_plan.csv`). Each drone has a series of semicolon-separated instructions.

**Example line for Drone ID 1:**

```
1,9,0,0,UP;UP;SHAKE;SHAKE;UP;UP;UP;UP;LEFT;LEFT;LEFT;LEFT
```

**Explanation of Instructions:**

* `UP`: Increments Z coordinate by 1.
* `DOWN`: Decrements Z coordinate by 1.
* `LEFT`: Decrements X coordinate by 1.
* `RIGHT`: Increments X coordinate by 1.
* `FORWARD`: Increments Y coordinate by 1.
* `BACKWARD`: Decrements Y coordinate by 1.
* `SHAKE`: No change in position, simulates action.
* `ROTATE`: No change in position, simulates rotation.

---

## 3. Approach Followed for Each User Story

### US361 – Initialize hybrid simulation environment

* **Shared Memory & Semaphores:** Parent creates a `SharedMemoryLayout` segment and two POSIX semaphores per drone (`sem_open`, `sem_unlink`).
* **Process & Thread Creation:** Parent forks one child per drone, and spawns three threads (simulation loop, collision detection, report generation).

### US362 – Function-specific threads in the parent process

* **Simulation Thread:** Drives the step loop: signals drones, waits for acknowledgments, logs updates, displays UI.
* **Collision Thread:** Waits for step completion signal (`step_cond`), scans shared memory positions, logs collisions, signals report thread.
* **Report Thread:** Waits for collision signal (`collision_cond`), logs collision details as they occur.

### US363 – Notify report thread via condition variables upon collision

* **Collision Detection Thread:** Upon detecting at least one collision in a step, populates a `CollisionInfo` struct and sets `collision_event_occurred = 1` inside `data_mutex` lock, then `pthread_cond_signal(&collision_cond)`.
* **Report Thread:** Inside `data_mutex`, waits on `collision_cond`, and immediately logs the collision using `log_collision_to_report()` with the stored `last_collision_info`.

### US364 – Step-by-step synchronization with semaphores

* **Lockstep Protocol:** Simulation thread posts each drone’s `sem_child_can_act` to allow one instruction. Each drone waits on its semaphore, executes an instruction, updates shared memory, then posts its `sem_parent_can_read`. Parent waits on each `sem_parent_can_read` before proceeding.

### US365 – Generate and store final simulation report

* **Per-Step Logging:** Throughout simulation, `reporting.c` functions record initial states, drone updates, finishes, and collisions to `simulation_report.txt`.
* **Final Summary:** After threads join, `log_simulation_summary_to_report()` writes total drones, time steps, collision log, final statuses (read from shared memory), and overall status. The report is flushed and closed.

---

## 4. Self-Assessment of Commitment (0 – 100 %)

All group members contributed equally to Sprint 2 solution: implementation, testing, design, and documentation.

| Group Member   | Student Number | Contribution (%) |
| -------------- | -------------- | ---------------- |
| Amir Masnavi   | 1221579        | 25 %             |
| Leonor Marinho | 1230977        | 25 %             |
| Inês Oliveira  | 1231205        | 25 %             |
| Rui Moreira    | 1221696        | 25 %             |
