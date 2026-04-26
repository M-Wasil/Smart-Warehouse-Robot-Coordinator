# 🤖📦 Smart Warehouse Robot Coordinator

A high-performance, multithreaded simulation of autonomous warehouse robots, engineered in **C99** using **POSIX Threads** and **Raylib**. This project demonstrates advanced Operating Systems concepts through a real-time, interactive grid-based logistics environment.

## 🌟 Project Overview

The Smart Warehouse Robot Coordinator simulates a fleet of robots navigating a 5x5 grid. Robots are tasked with picking up items from the "Floor" and delivering them to "Shelves," all while managing shared resources and avoiding physical collisions in a concurrent environment.

### 📍 Current Project Status
- **Stable Core:** Multithreaded navigation engine and collision avoidance are fully functional.
- **GUI Revamp:** An interactive Raylib-based interface has been implemented, providing real-time thread state visualization.
- **Task Management:** A priority-based task queue with starvation prevention (aging) is operational.
- **Bottleneck Handling:** Semaphore-regulated Critical Zone management is active to optimize throughput.

### Key Components:
- **Shelves (Row 0):** Delivery targets for items.
- **Floor (Rows 1-3):** Active workspace where items are spawned via user interaction.
- **Docks (Row 4):** Home base for robots when idle.
- **Critical Zone (Cols 2-3):** A high-traffic bottleneck managed by semaphores.

## 🧠 Core OS Concepts Demonstrated

### 🧵 Multithreading & Concurrency
- **POSIX Threads (`pthread`):** Each robot is a fully independent thread with its own state machine and decision-making logic.
- **Main GUI Thread:** Handles rendering and user input without blocking worker threads, using a private state snapshot to prevent priority inversion.

### 🔒 Synchronization & Resource Management
- **Mutexes (Mutual Exclusion):** 
  - **Grid Mutexes:** Every cell in the 5x5 grid has its own mutex. Robots must acquire a cell's mutex before entering, ensuring no two robots ever occupy the same space.
  - **Task Queue Mutex:** Protects the shared priority queue of pending tasks.
  - **Stats Mutex:** Ensures thread-safe updates to global performance metrics.
- **Counting Semaphores:** Used to regulate traffic in the **Critical Zone** (Columns 2 & 3). A maximum of 2 robots are permitted in this zone simultaneously to prevent gridlocks.

### 🛡️ Deadlock & Starvation Prevention
- **Collision Avoidance:** Robots use `pthread_mutex_timedlock`. If a path is blocked, they attempt to "dodge" into adjacent cells or wait/retry, breaking potential circular waits.
- **Priority Aging:** The task scheduler implements aging. As tasks sit in the queue, their priority dynamically increases, ensuring low-priority tasks are eventually serviced even during high-demand periods.

## 🎨 Interactive GUI Legend

The simulation provides real-time visual feedback on the internal state of each POSIX thread:

* 🟢 **Green (MOVING):** The thread is active and navigating the grid.
* 🔴 **Red (WAIT ZONE):** The thread is `BLOCKED` by the Critical Zone semaphore.
* 🟠 **Orange (WAIT CELL):** The thread is `BLOCKED` awaiting a grid cell mutex.
* ⚫ **Grey (IDLE):** The thread is suspended, waiting for new tasks to be spawned.

## 🚀 Getting Started

### Prerequisites

You will need a C compiler and standard development libraries. On Ubuntu/WSL:

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git libasound2-dev libx11-dev libxrandr-dev libxi-dev libgl1-mesa-dev libxcursor-dev libxinerama-dev
```

### Installation & Build

1. **Install Raylib Locally:**
   The project includes a helper to build the graphics engine without system-wide changes.
   ```bash
   make install-raylib
   ```

2. **Compile the GUI Version:**
   ```bash
   make gui
   ```

3. **Launch the Simulation:**
   ```bash
   ./warehouse_gui
   ```

### Controls
- **Left Click:** Spawn a pickup task on any Floor cell (Rows 1-3).
- **Close Window:** Terminates the simulation and generates a performance report in the terminal.

## 📊 Performance Reporting

Upon exit, the system generates a detailed report including:
- Total task throughput (tasks per second).
- Per-robot efficiency metrics.
- Resource contention statistics (Zone blocks and Collision waits).
- Detailed event logs in `logs.txt`.

---

Developed as a practical exploration of POSIX synchronization and concurrent system design.
