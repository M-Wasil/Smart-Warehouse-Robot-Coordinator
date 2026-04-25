# Smart Warehouse Robot Coordinator

## Description
A multithreaded CLI simulation of autonomous warehouse robots demonstrating operating system synchronization concepts with POSIX threads, mutexes, and semaphores.

## Features
- Robot threads (POSIX `pthread`)
- Mutex-protected queue, item table, and shared occupancy state
- Semaphore-controlled critical zone capacity
- Deadlock prevention via global lock ordering + timeout/retry
- Collision-aware next-cell reservation before move commit
- Priority task queue with atomic item claim/complete transitions
- Hybrid CLI demo:
  - detailed event timeline logs
  - periodic live status line
  - final performance report

## Technologies
- C (C99)
- POSIX Threads (`pthread`)
- POSIX Semaphores

## How to Run
```bash
make
./warehouse_sim
```

Or:

```bash
make run
```

## CLI Demo Walkthrough

The runtime output has two layers:

1. **Event timeline logs** (structured tags):
   - `[task_claimed]`
   - `[zone_wait]`
   - `[collision_wait]`
   - `[deadlock_retry]`
   - `[move_commit]`
   - `[task_completed]`
   - `[reroute]`
2. **Periodic status lines**:
   - queue depth
   - active robots
   - blocked robots
   - zone usage
   - completed tasks

At the end, the simulation prints:
- total tasks completed
- runtime and throughput
- average wait time
- zone congestion and utilization events
- collision wait totals
- per-robot completion and wait counters

## Stress Testing

To vary simulation load, change constants in `main.c`:
- `NUM_ROBOTS`
- `NUM_TASKS`
- `SIM_SECONDS`

Then rebuild and run:

```bash
make clean && make
./warehouse_sim
```
