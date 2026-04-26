/* ================================================================
   main_gui.c  --  Entry point for the revamped Warehouse Simulator

   Layout (5x5 grid):
     Row 0 (y=0, top)    : SHELVES  -- items are delivered here
     Row 1-3 (y=1-3)     : FLOOR    -- navigation / item spawning
     Row 4 (y=4, bottom) : DOCKS    -- robots park here when idle

   Coordinate system:  x=column (0..4), y=row (0..4)
   cellOccupancy[y][x]  (row first, col second)

   Interaction:
     Left-click any FLOOR cell (row 1-3) to spawn a new item there.
     The robot will pick it up and deliver it to a free shelf in row 0.
     Click on row 0 or row 4 is ignored (shelves / docks).
   ================================================================ */

#define _DEFAULT_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>

#include "robot.h"
#include "gui.h"
#include "raylib.h"

#define NUM_ROBOTS 3
/* Robot dock columns: placed at columns 0, 2, 4 of row 4 */
static const int DOCK_COLS[NUM_ROBOTS] = {0, 2, 4};

/* ----------------------------------------------------------------
   screen_to_grid: pixel -> (gx=col, gy=row), returns 1 if inside grid
   ---------------------------------------------------------------- */
static int screen_to_grid(int mx, int my, int *gx, int *gy) {
    int step = CELL_PX + GAP_PX;
    if (mx < GRID_OFF_X || my < GRID_OFF_Y) return 0;
    int col = (mx - GRID_OFF_X) / step;
    int row = (my - GRID_OFF_Y) / step;
    if (col < 0 || col >= COLS || row < 0 || row >= ROWS) return 0;
    int lx = (mx - GRID_OFF_X) % step;
    int ly = (my - GRID_OFF_Y) % step;
    if (lx >= CELL_PX || ly >= CELL_PX) return 0;
    *gx = col; *gy = row;
    return 1;
}

/* ----------------------------------------------------------------
   find_free_shelf: pick a random un-occupied shelf column (y=0).
   Returns -1 if no free shelf exists.
   ---------------------------------------------------------------- */
static int find_free_shelf(Warehouse *wh) {
    /* Collect free shelf columns */
    int free[COLS], n = 0;
    pthread_mutex_lock(&wh->stateMutex);
    for (int x = 0; x < COLS; x++) {
        if (wh->cellOccupancy[0][x] == -1)
            free[n++] = x;
    }
    pthread_mutex_unlock(&wh->stateMutex);
    if (n == 0) return -1;
    return free[rand() % n];
}

/* ----------------------------------------------------------------
   spawn_task: create item at (pickX, pickY) and target a shelf.
   Thread-safe.
   ---------------------------------------------------------------- */
static void spawn_task(Warehouse *wh, int pickX, int pickY) {
    /* Only allow spawning on floor rows (1..3) */
    if (pickY < 1 || pickY > 3) {
        printf("[Main] Click ignored: only floor rows 1-3 are valid.\n");
        return;
    }

    int shelfCol = find_free_shelf(wh);
    if (shelfCol < 0) {
        printf("[Main] All shelves are occupied -- cannot spawn task.\n");
        return;
    }

    pthread_mutex_lock(&wh->taskMutex);
    if (wh->itemCount >= MAX_ITEMS || wh->taskQueue.size >= MAX_TASKS) {
        pthread_mutex_unlock(&wh->taskMutex);
        printf("[Main] Queue full -- ignoring click.\n");
        return;
    }
    int newId = wh->itemCount;

    wh->items[newId].id        = newId;
    wh->items[newId].x         = pickX;
    wh->items[newId].y         = pickY;
    wh->items[newId].available = 1;
    wh->items[newId].claimed   = 0;
    wh->items[newId].completed = 0;
    wh->itemCount++;
    pthread_mutex_unlock(&wh->taskMutex);

    Task t;
    t.id          = newId + 1000;
    t.itemId      = newId;
    t.pickupX     = pickX;
    t.pickupY     = pickY;
    t.dropX       = shelfCol;
    t.dropY       = 0;           /* shelf row */
    t.priority    = rand() % 5 + 1;
    t.deadline    = rand() % 10 + 5;
    t.enqueueTime = (long)time(NULL);

    pushTask(wh, t);
    printf("[Main] Item %d spawned at floor (%d,%d) -> shelf (%d,0)\n",
           newId, pickX, pickY, shelfCol);
}

/* ================================================================
   main
   ================================================================ */
int main(void) {

    /* ---- 1. Init warehouse ---- */
    Warehouse wh;
    initWarehouse(&wh);
    srand((unsigned int)time(NULL));

    /* ---- 2. Open Raylib window ---- */
    gui_init();

    /* ---- 3. Spawn robot threads, docked at bottom row ---- */
    pthread_t threads[NUM_ROBOTS];
    Robot     robots[NUM_ROBOTS];
    time_t    simStart = time(NULL);

    for (int i = 0; i < NUM_ROBOTS; i++) {
        int dx = DOCK_COLS[i];
        int dy = ROWS - 1;          /* row 4 = bottom */

        robots[i].id    = i + 1;
        robots[i].wh    = &wh;
        robots[i].curX  = dx;
        robots[i].curY  = dy;
        robots[i].dockX = dx;
        robots[i].dockY = dy;
        robots[i].hasItem = 0;

        /* Reserve initial cell */
        wh.cellOccupancy[dy][dx] = robots[i].id;
        pthread_mutex_lock(&wh.gridMutex[dy][dx]);

        /* Robot 2 starts at column 2 (CZ) -- pre-acquire one CZ token */
        if (dx >= 2 && dx <= 3)
            sem_wait(&wh.zoneSemaphore);

        if (pthread_create(&threads[i], NULL, robotFunc, &robots[i]) != 0) {
            fprintf(stderr, "[Main] ERROR: pthread_create failed for Robot-%d\n", i+1);
            wh.running = 0;
            gui_close();
            return 1;
        }
    }

    printf("[Main] %d robots docked at row 4. Click floor cells to spawn tasks.\n",
           NUM_ROBOTS);

    /* ---- 4. GUI render loop ---- */
    GuiFrame frame;
    int hoverGX = -1, hoverGY = -1;

    while (!WindowShouldClose()) {

        Vector2 mp = GetMousePosition();
        int gx, gy;
        if (screen_to_grid((int)mp.x, (int)mp.y, &gx, &gy)) {
            hoverGX = gx; hoverGY = gy;
        } else {
            hoverGX = -1; hoverGY = -1;
        }

        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && wh.running) {
            if (screen_to_grid((int)mp.x, (int)mp.y, &gx, &gy))
                spawn_task(&wh, gx, gy);
        }

        gui_sample_frame(&wh, NUM_ROBOTS, &frame, simStart);
        gui_draw_frame(&frame, wh.running, hoverGX, hoverGY);
    }

    /* ---- 5. Stop and join ---- */
    wh.running = 0;
    printf("\n[Main] Window closed -- joining threads...\n");
    for (int i = 0; i < NUM_ROBOTS; i++) {
        pthread_join(threads[i], NULL);
        printf("[Main] Robot-%d joined.\n", i + 1);
    }

    /* ---- 6. Report ---- */
    double timeTaken = difftime(time(NULL), simStart);
    printf("\n=== PERFORMANCE REPORT ===\n");
    printf("Tasks completed : %d\n",    wh.totalTasksCompleted);
    printf("Time            : %.1f s\n", timeTaken);
    if (timeTaken > 0)
        printf("Throughput      : %.2f t/s\n", wh.totalTasksCompleted / timeTaken);
    printf("Zone blocks     : %d\n",    wh.zoneBlockCount);
    printf("Collision waits : %d\n",    wh.totalCollisionWaits);
    for (int i = 0; i < NUM_ROBOTS; i++)
        printf("  Robot-%d  done=%d  zone_w=%d  cell_w=%d\n",
               i+1, wh.robotTasksCompleted[i],
               wh.robotZoneWaits[i], wh.robotCollisionWaits[i]);

    /* ---- 7. Cleanup ---- */
    gui_close();
    if (wh.logFile) { fprintf(wh.logFile, "\n=== Simulation ended ===\n"); fclose(wh.logFile); }
    printf("[Main] Done. See logs.txt\n");
    return 0;
}
