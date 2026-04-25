# 🤖📦 Smart Warehouse Robot Coordinator

A multithreaded simulation of autonomous warehouse robots, built in C (C99) using **POSIX Threads** and **Raylib**. 

This project was developed for our Operating Systems course to practically demonstrate core OS synchronization concepts. It simulates a smart warehouse where multiple robots (threads) move concurrently to pick and place items while avoiding physical collisions and spatial deadlocks.

## 🧠 Core OS Concepts Demonstrated

* **POSIX Multithreading (`pthread`):** Each robot operates as an independent, concurrent worker thread.
* **Mutual Exclusion (Mutexes):** Used to protect shared resources, including the task queue, logging stream, and individual grid cells (`cellOccupancy`).
* **Counting Semaphores:** Used to strictly limit the number of threads permitted inside the high-traffic "Critical Zone" (max 2 robots), preventing congestion.
* **Deadlock Prevention:** Implements strict ordered resource acquisition. When a robot needs to lock multiple grid cells, it mathematically locks the lower-indexed cell first, breaking the **Circular Wait** (Coffman) condition.
* **Starvation Prevention (Aging):** Dynamic priority boosting in the task scheduler ensures lower-priority threads aren't permanently starved by high-priority influxes.
* **Read-Only Observer Pattern (GUI):** The main GUI thread samples the shared state via fast mutex `memcpy` operations and renders from a private snapshot. This prevents **Priority Inversion**, ensuring the GPU rendering never blocks the worker threads.

## 🎨 Visual Legend (Thread States)

The GUI visualizes the exact OS state of each POSIX thread in real-time:
* 🟢 **Green (MOVING):** Thread is currently in the `RUNNING` state on the CPU.
* 🔴 **Red (WAIT ZONE):** Thread is `BLOCKED` on the counting semaphore.
* 🟠 **Orange (WAIT CELL):** Thread is `BLOCKED` on a cell mutex.
* ⚫ **Grey (IDLE):** Thread is sleeping or yielded, waiting for tasks.

---

## 🚀 How to Run on Ubuntu / WSL

The project includes an automated script in the `Makefile` to download and compile the graphics engine (Raylib) locally, so you don't need to mess with system-wide library installations.

### Step 1: Install System Dependencies
Open your terminal and install the required C compilers and graphics libraries:
```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git libasound2-dev libx11-dev libxrandr-dev libxi-dev libgl1-mesa-dev libxcursor-dev libxinerama-dev
```
Step 2: Build the Local Graphics Library
Run the automated Makefile target to clone and compile Raylib directly into the project folder. (This only needs to be done once and takes about 1-2 minutes).
```bash
make install-raylib
```
Step 3: Compile and Run the GUI
Compile the GUI version of the simulator:
```bash
make gui
```
Launch the simulation:
```bash
./warehouse_gui
```
💻 Running the Pure CLI Backend (No GUI)
If you are working on a headless server or simply want to test the pure multithreaded backend without the graphics window, you can run the CLI version:
```bash
make clean
make all
./warehouse_sim
```
This will run the simulation directly in the terminal, printing real-time concurrency logs and a final performance/throughput report.
