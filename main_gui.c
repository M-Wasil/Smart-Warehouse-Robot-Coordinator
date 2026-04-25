/* ================================================================
   main_gui.c  --  Entry point for GUI-enabled Warehouse Simulator

   Architecture:
     - main() runs on the PRIMARY thread (Raylib MUST be driven from
       the same thread that called InitWindow).
     - Robot worker threads are spawned with pthread_create as before.
     - The main thread runs a non-blocking render loop:
         while (!WindowShouldClose()) {
             gui_sample_frame(...)  // brief lock to copy state
             gui_draw_frame(...)    // render from private copy (no lock)
         }
     - After the window closes (user presses Esc or Ã— button),
       wh.running is set to 0 so all robot threads exit cleanly.
     - pthread_join() then collects each thread before cleanup.

   Why this design is safe (OS viva answer):
     The GUI never holds a mutex while calling any Raylib function.
     Raylib may call into the OS (OpenGL, window system) and could
     block.  Holding a mutex while doing OS I/O risks priority
     inversion and contention.  We copy data OUT under the mutex,
     release immediately, and then draw from the local copy.

   Compile:
     gcc -std=c99 -Wall -Wextra -g \
         main_gui.c warehouse.c robot.c task.c gui.c \
         -o warehouse_gui \
         -pthread -lm -lraylib -lGL -lm -lpthread -ldl -lrt -lX11

   Or use:  make gui
   ================================================================ */

#define _DEFAULT_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>

#include "robot.h"
#include "gui.h"

#define NUM_ROBOTS   3
#define NUM_TASKS   15
#define SIM_SECONDS 30   /* window stays open for up to this long */

int main(void) {

    /* ---- 1. Initialise shared warehouse ------------------------ */
    Warehouse wh;
    initWarehouse(&wh);
    srand((unsigned int)time(NULL));

    /* ---- 2. Seed item-backed tasks ----------------------------- */
    for (int i = 0; i < NUM_TASKS; i++) {
        Task t;
        t.id        = i;
        t.itemId    = i;
        t.pickupX   = rand() % COLS;
        t.pickupY   = rand() % ROWS;
        t.dropX     = rand() % COLS;
        t.dropY     = rand() % ROWS;
        t.priority  = rand() % 5 + 1;
        t.deadline  = rand() % 10 + 5;

        wh.items[i].id        = i;
        wh.items[i].x         = t.pickupX;
        wh.items[i].y         = t.pickupY;
        wh.items[i].available = 1;
        wh.items[i].claimed   = 0;
        wh.items[i].completed = 0;
        wh.itemCount++;

        pushTask(&wh, t);
    }

    /* ---- 3. Open Raylib window --------------------------------- */
    /*
     * MUST be called from the main thread before spawning robots.
     * Raylib's OpenGL context is bound to this thread.
     */
    gui_init();

    /* ---- 4. Spawn robot threads -------------------------------- */
    pthread_t threads[NUM_ROBOTS];
    Robot     robots[NUM_ROBOTS];
    time_t    simStart = time(NULL);

    for (int i = 0; i < NUM_ROBOTS; i++) {
        robots[i].id = i + 1;
        robots[i].wh = &wh;
        if (pthread_create(&threads[i], NULL, robotFunc, &robots[i]) != 0) {
            fprintf(stderr, "[Main] ERROR: pthread_create failed for Robot-%d\n",
                    i + 1);
            wh.running = 0;
            gui_close();
            return 1;
        }
    }

    printf("[Main] %d robot threads spawned.  Rendering GUI...\n", NUM_ROBOTS);
    printf("[Main] Close the window or wait %d seconds to stop.\n", SIM_SECONDS);

    /* ---- 5. GUI render loop ------------------------------------ */
    /*
     * Non-blocking main loop.
     * Each iteration:
     *   a) sample_frame  : acquires/releases mutexes briefly
     *   b) draw_frame    : pure rendering, no locks held
     *
     * WindowShouldClose() returns true when the user closes the
     * window OR presses Escape.
     *
     * We also check elapsed time so the sim auto-stops after
     * SIM_SECONDS even if the window stays open.
     */
    GuiFrame frame;
    while (!WindowShouldClose()) {
        double elapsed = difftime(time(NULL), simStart);
        int    running = wh.running && (elapsed < (double)SIM_SECONDS);

        if (!running && wh.running) {
            /* Time limit reached -- signal robots to stop */
            wh.running = 0;
        }

        /* Sample shared state under minimal lock hold time */
        gui_sample_frame(&wh, NUM_ROBOTS, &frame, simStart);

        /* Draw from private copy (no locks held during rendering) */
        gui_draw_frame(&frame, wh.running);
    }

    /* ---- 6. Signal stop and join all threads ------------------- */
    wh.running = 0;
    printf("\n[Main] Window closed.  Waiting for robot threads...\n");

    for (int i = 0; i < NUM_ROBOTS; i++) {
        pthread_join(threads[i], NULL);
        printf("[Main] Robot-%d joined.\n", i + 1);
    }

    /* ---- 7. Final performance report (console) ----------------- */
    double timeTaken = difftime(time(NULL), simStart);
    printf("\n=== PERFORMANCE REPORT ===\n");
    printf("Tasks completed : %d\n",   wh.totalTasksCompleted);
    printf("Time            : %.2f s\n", timeTaken);
    if (timeTaken > 0.0)
        printf("Throughput      : %.2f tasks/sec\n",
               wh.totalTasksCompleted / timeTaken);
    if (wh.totalTasksCompleted > 0)
        printf("Avg wait        : %.2f s\n",
               wh.totalWaitTime / wh.totalTasksCompleted);
    printf("Zone blocks     : %d\n",   wh.zoneBlockCount);
    printf("Collision waits : %d\n",   wh.totalCollisionWaits);

    printf("\nPer-robot report:\n");
    for (int i = 0; i < NUM_ROBOTS; i++) {
        printf("  Robot-%d  done=%d  zone_waits=%d  cell_waits=%d\n",
               i + 1,
               wh.robotTasksCompleted[i],
               wh.robotZoneWaits[i],
               wh.robotCollisionWaits[i]);
    }

    /* ---- 8. Cleanup -------------------------------------------- */
    gui_close();

    if (wh.logFile) {
        fprintf(wh.logFile, "\n=== Simulation ended ===\n");
        fclose(wh.logFile);
    }

    printf("[Main] Done.  See logs.txt for full log.\n");
    return 0;
}
