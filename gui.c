/* ================================================================
   gui.c  --  Raylib GUI for the Smart Warehouse Robot Coordinator

   Design principles:
     1. GUI thread NEVER writes to Warehouse fields.
     2. Locks are held only during the "sample" phase, not during
        the "draw" phase (lock â†’ copy â†’ unlock â†’ draw).
     3. All drawing uses basic Raylib primitives (rectangles,
        circles, text) -- no textures, no external assets.
     4. The render loop is non-blocking: WindowShouldClose() and
        BeginDrawing()/EndDrawing() do not sleep.

   OS concept mapping (important for viva):
     - The GUI loop is effectively a high-priority "monitor thread"
       that observes shared state using the SAME mutexes the robot
       threads use for writes.  This is safe because Raylib renders
       from a private GuiFrame copy, so robots can keep running
       while the GPU draws the previous frame.
     - cellOccupancy and robotState are protected by stateMutex;
       stats by statsMutex; task queue by taskMutex.  We acquire
       each only briefly to memcpy the relevant fields.
   ================================================================ */

#include "gui.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>

/* ================================================================
   Colour palette  (dark industrial theme suits an OS demo)
   ================================================================ */
static const Color C_BG          = { 18,  20,  30, 255 }; /* near-black bg    */
static const Color C_GRID_EMPTY  = { 38,  42,  60, 255 }; /* dark cell        */
static const Color C_GRID_CZ     = { 60,  35,  20, 255 }; /* critical zone    */
static const Color C_GRID_OCC    = { 50,  60,  90, 255 }; /* occupied cell    */
static const Color C_ITEM        = {220, 180,  60, 255 }; /* yellow item dot  */
static const Color C_PANEL_BG    = { 25,  28,  45, 255 }; /* right panel bg   */
static const Color C_BORDER      = { 70,  80, 110, 255 }; /* grid border      */
static const Color C_TEXT        = {210, 215, 230, 255 }; /* normal text      */
static const Color C_TEXT_DIM    = {120, 130, 155, 255 }; /* dimmed text      */
static const Color C_ACCENT      = { 80, 160, 255, 255 }; /* blue accent      */
static const Color C_TITLE       = {255, 200,  80, 255 }; /* title gold       */

/* Robot state colours (OS analogy: thread state colours) */
static const Color C_MOVING      = { 60, 220,  90, 255 }; /* green  = running */
static const Color C_WAIT_ZONE   = {230,  70,  70, 255 }; /* red    = blocked */
static const Color C_WAIT_CELL   = {230, 120,  40, 255 }; /* orange = blocked */
static const Color C_IDLE        = {160, 165, 190, 255 }; /* grey   = idle    */

/* ================================================================
   gui_init
   Open Raylib window.  Must be called from the main thread before
   any other Raylib calls.
   ================================================================ */
void gui_init(void) {
    SetTraceLogLevel(LOG_WARNING);   /* suppress verbose Raylib info logs */
    InitWindow(WIN_W, WIN_H, "Smart Warehouse Robot Coordinator");
    SetTargetFPS(FPS);
}

/* ================================================================
   gui_close
   ================================================================ */
void gui_close(void) {
    CloseWindow();
}

/* ================================================================
   gui_cell_center
   Convert a grid (col, row) to pixel coordinates of the cell centre.

   Layout:
     col 0 left edge  = GRID_OFF_X
     row 0 top  edge  = GRID_OFF_Y
     cell width/height = CELL_PX + GAP_PX
   ================================================================ */
Vector2 gui_cell_center(int gridX, int gridY) {
    Vector2 v;
    v.x = (float)(GRID_OFF_X + gridX * (CELL_PX + GAP_PX) + CELL_PX / 2);
    v.y = (float)(GRID_OFF_Y + gridY * (CELL_PX + GAP_PX) + CELL_PX / 2);
    return v;
}

/* ================================================================
   robot_color
   Map a RobotState enum to a display colour.

   OS parallel:
     ROBOT_IDLE            â†’ thread in TASK_INTERRUPTIBLE (sleeping)
     ROBOT_MOVING          â†’ thread RUNNING on CPU
     ROBOT_WAITING_FOR_ZONEâ†’ thread blocked on semaphore (TASK_UNINTERRUPTIBLE)
     ROBOT_WAITING_FOR_CELLâ†’ thread blocked on mutex    (TASK_UNINTERRUPTIBLE)
   ================================================================ */
static Color robot_color(RobotState s) {
    switch (s) {
        case ROBOT_MOVING:          return C_MOVING;
        case ROBOT_WAITING_FOR_ZONE:return C_WAIT_ZONE;
        case ROBOT_WAITING_FOR_CELL:return C_WAIT_CELL;
        case ROBOT_IDLE:
        default:                    return C_IDLE;
    }
}

static const char* robot_state_name(RobotState s) {
    switch (s) {
        case ROBOT_MOVING:           return "MOVING";
        case ROBOT_WAITING_FOR_ZONE: return "WAIT ZONE";
        case ROBOT_WAITING_FOR_CELL: return "WAIT CELL";
        case ROBOT_IDLE:
        default:                     return "IDLE";
    }
}

/* ================================================================
   gui_sample_frame
   Atomically snapshot all shared state the GUI needs.

   Locking strategy:
     We acquire each mutex independently (not all at once) to avoid
     holding multiple locks simultaneously, which would increase
     contention with the robot threads.

     Order:
       1. stateMutex  â†’ copy cellOccupancy + robotState
       2. statsMutex  â†’ copy performance counters
       3. taskMutex   â†’ copy queue depth + item table

   This is safe because the GUI only READS -- it does not modify
   any Warehouse field -- so it never contributes to deadlock.
   ================================================================ */
void gui_sample_frame(Warehouse *wh, int numRobots, GuiFrame *f,
                      time_t simStart) {
    int i, r, c;

    /* ---- 1. Sample grid occupancy + robot states (stateMutex) -- */
    pthread_mutex_lock(&wh->stateMutex);

    for (r = 0; r < ROWS; r++)
        for (c = 0; c < COLS; c++)
            f->cellOcc[r][c] = wh->cellOccupancy[r][c];

    f->numRobots = (numRobots < MAX_ROBOTS) ? numRobots : MAX_ROBOTS;
    for (i = 0; i < f->numRobots; i++) {
        f->robots[i].id    = i + 1;
        f->robots[i].state = wh->robotState[i];
    }

    pthread_mutex_unlock(&wh->stateMutex);

    /* ---- 2. Sample performance stats (statsMutex) -------------- */
    pthread_mutex_lock(&wh->statsMutex);

    f->totalDone      = wh->totalTasksCompleted;
    f->zoneInUse      = wh->zoneInUse;
    f->zoneBlockCount = wh->zoneBlockCount;
    f->collisionWaits = wh->totalCollisionWaits;
    f->totalWaitTime  = wh->totalWaitTime;

    for (i = 0; i < f->numRobots; i++) {
        f->robots[i].tasksCompleted  = wh->robotTasksCompleted[i];
        f->robots[i].zoneWaits       = wh->robotZoneWaits[i];
        f->robots[i].collisionWaits  = wh->robotCollisionWaits[i];
    }

    pthread_mutex_unlock(&wh->statsMutex);

    /* ---- 3. Sample item table + queue depth (taskMutex) -------- */
    pthread_mutex_lock(&wh->taskMutex);

    f->itemCount = (wh->itemCount < MAX_ITEMS) ? wh->itemCount : MAX_ITEMS;
    for (i = 0; i < f->itemCount; i++) {
        f->itemAvail[i] = wh->items[i].available;
        f->itemX[i]     = wh->items[i].x;
        f->itemY[i]     = wh->items[i].y;
    }
    f->queueDepth = wh->taskQueue.size;

    pthread_mutex_unlock(&wh->taskMutex);

    /* ---- 4. Elapsed time (no lock needed -- simStart is const) - */
    f->elapsedSec = difftime(time(NULL), simStart);
}

/* ================================================================
   draw_grid
   Draw the 5x5 warehouse grid.

   Visual encoding:
     - Dark blue cell   = empty normal cell
     - Dark orange cell = empty Critical Zone cell (columns 2-3)
     - Brighter cell    = cell currently occupied by a robot
     - Yellow dot       = unclaimed item sitting on the floor
   ================================================================ */
static void draw_grid(const GuiFrame *f) {
    int r, c, i;

    /* Column range of the Critical Zone (mirrors warehouse.h logic) */
    /* CZ is defined in warehouse.h but since we can't call the
       warehouse function from pure GUI code we replicate the range.
       Critical Zone = columns where X >= 2 and X <= 3 on a 5x5 grid.
       (Adjust these constants to match your warehouse.h CZ_X_MIN/MAX) */
    const int CZ_MIN = 2;
    const int CZ_MAX = 3;

    /* --- Draw section label --- */
    DrawText("WAREHOUSE GRID", GRID_OFF_X, 50, 16, C_ACCENT);

    /* --- Draw column numbers above grid --- */
    for (c = 0; c < COLS; c++) {
        char label[4];
        snprintf(label, sizeof(label), "%d", c);
        int lx = GRID_OFF_X + c * (CELL_PX + GAP_PX) + CELL_PX/2 - 4;
        DrawText(label, lx, GRID_OFF_Y - 20, 14, C_TEXT_DIM);
    }
    /* --- Draw row numbers left of grid --- */
    for (r = 0; r < ROWS; r++) {
        char label[4];
        snprintf(label, sizeof(label), "%d", r);
        int ly = GRID_OFF_Y + r * (CELL_PX + GAP_PX) + CELL_PX/2 - 7;
        DrawText(label, GRID_OFF_X - 18, ly, 14, C_TEXT_DIM);
    }

    /* --- Draw cells --- */
    for (r = 0; r < ROWS; r++) {
        for (c = 0; c < COLS; c++) {
            int px = GRID_OFF_X + c * (CELL_PX + GAP_PX);
            int py = GRID_OFF_Y + r * (CELL_PX + GAP_PX);

            /* Choose cell background colour */
            Color bg;
            int occ = f->cellOcc[r][c];
            int inCZ = (c >= CZ_MIN && c <= CZ_MAX);

            if (occ != -1) {
                bg = C_GRID_OCC;          /* robot is here */
            } else if (inCZ) {
                bg = C_GRID_CZ;           /* critical zone */
            } else {
                bg = C_GRID_EMPTY;        /* normal empty  */
            }

            /* Draw cell rectangle */
            DrawRectangle(px, py, CELL_PX, CELL_PX, bg);
            /* Draw cell border */
            DrawRectangleLines(px, py, CELL_PX, CELL_PX, C_BORDER);

            /* Draw "CZ" label inside Critical Zone cells */
            if (inCZ && occ == -1) {
                DrawText("CZ", px + CELL_PX/2 - 10, py + CELL_PX/2 - 7,
                         14, (Color){200, 100, 50, 120});
            }

            /* Draw coordinate label (small, top-left of cell) */
            char coord[8];
            snprintf(coord, sizeof(coord), "%d,%d", c, r);
            DrawText(coord, px + 4, py + 4, 10, C_TEXT_DIM);
        }
    }

    /* --- Draw items (yellow dots on floor) --- */
    for (i = 0; i < f->itemCount; i++) {
        if (!f->itemAvail[i]) continue;  /* already picked or done */
        int ix = f->itemX[i];
        int iy = f->itemY[i];
        if (ix < 0 || ix >= COLS || iy < 0 || iy >= ROWS) continue;

        Vector2 centre = gui_cell_center(ix, iy);
        DrawCircleV(centre, 8.0f, C_ITEM);
        DrawCircleLines((int)centre.x, (int)centre.y, 8, (Color){255,240,100,200});
    }

    /* --- Critical Zone legend box --- */
    int legX = GRID_OFF_X;
    int legY = GRID_OFF_Y + ROWS * (CELL_PX + GAP_PX) + 12;
    DrawRectangle(legX, legY, 16, 12, C_GRID_CZ);
    DrawText("= Critical Zone (sem, max 2 robots)", legX + 22, legY, 13, C_TEXT_DIM);
}

/* ================================================================
   draw_robots
   Draw each robot as a coloured circle with its ID.

   Placement logic:
     - If a robot's last known occupied cell is visible we place it
       exactly at that cell centre.
     - If not (robot between cells), we fall back to a fixed
       "home" position in a row below the grid so it's always visible.

   The colour encodes the OS thread state (see robot_color()).
   ================================================================ */
static void draw_robots(const GuiFrame *f) {
    int i, r, c;

    DrawText("ROBOTS", GRID_OFF_X,
             GRID_OFF_Y + ROWS*(CELL_PX+GAP_PX) + 40, 16, C_ACCENT);

    for (i = 0; i < f->numRobots; i++) {
        const GuiRobotSnapshot *rb = &f->robots[i];
        int rid = rb->id;

        /* Find which cell this robot occupies (scan cellOcc) */
        int foundR = -1, foundC = -1;
        for (r = 0; r < ROWS && foundR == -1; r++) {
            for (c = 0; c < COLS && foundR == -1; c++) {
                if (f->cellOcc[r][c] == rid) {
                    foundR = r;
                    foundC = c;
                }
            }
        }

        float px, py;
        if (foundR != -1) {
            Vector2 cv = gui_cell_center(foundC, foundR);
            px = cv.x;
            py = cv.y;
        } else {
            /* Not on grid -- show in robot home row below grid */
            int homeY = GRID_OFF_Y + ROWS*(CELL_PX+GAP_PX) + 55;
            px = (float)(GRID_OFF_X + (i % COLS) * (CELL_PX + GAP_PX) + CELL_PX/2);
            py = (float)(homeY + 20);
        }

        Color col = robot_color(rb->state);

        /* Outer glow ring (shows blockage more visibly) */
        if (rb->state == ROBOT_WAITING_FOR_ZONE ||
            rb->state == ROBOT_WAITING_FOR_CELL) {
            DrawCircleLines((int)px, (int)py, ROBOT_R + 4,
                            (Color){col.r, col.g, col.b, 100});
        }

        /* Main robot circle */
        DrawCircleV((Vector2){px, py}, (float)ROBOT_R, col);
        DrawCircleLines((int)px, (int)py, ROBOT_R, WHITE);

        /* Robot ID label inside circle */
        char idstr[4];
        snprintf(idstr, sizeof(idstr), "R%d", rid);
        int tw = MeasureText(idstr, 12);
        DrawText(idstr, (int)px - tw/2, (int)py - 6, 12, BLACK);
    }
}

/* ================================================================
   draw_stats_panel
   Right-hand panel showing live OS-level metrics.

   Each metric maps directly to a synchronization concept:
     zoneInUse      â†’ current semaphore value consumers
     zoneBlockCount â†’ sem_wait() contention count
     collisionWaits â†’ mutex contention count (cellOcc)
     queueDepth     â†’ scheduler run-queue depth
   ================================================================ */
static void draw_stats_panel(const GuiFrame *f, int simRunning) {
    int px = PANEL_X;
    int py = 40;
    int lh = 22;   /* line height */

    /* Panel background */
    DrawRectangle(px - 10, 30, PANEL_W + 10, WIN_H - 40, C_PANEL_BG);
    DrawRectangleLines(px - 10, 30, PANEL_W + 10, WIN_H - 40, C_BORDER);

    /* ---- Title ---- */
    DrawText("LIVE MONITOR", px, py, 18, C_TITLE);
    py += 28;

    /* ---- Simulation status ---- */
    const char *status = simRunning ? "[ RUNNING ]" : "[ STOPPED ]";
    Color sc = simRunning ? C_MOVING : C_WAIT_ZONE;
    DrawText(status, px, py, 15, sc);
    py += lh + 4;

    /* ---- Elapsed time ---- */
    char buf[128];
    snprintf(buf, sizeof(buf), "Elapsed: %.1f s", f->elapsedSec);
    DrawText(buf, px, py, 14, C_TEXT); py += lh;

    /* ---- Divider ---- */
    DrawLine(px, py, px + PANEL_W - 10, py, C_BORDER); py += 10;

    /* ---- Task stats ---- */
    DrawText("TASK SCHEDULER", px, py, 14, C_ACCENT); py += lh;

    snprintf(buf, sizeof(buf), "Completed : %d", f->totalDone);
    DrawText(buf, px, py, 13, C_TEXT); py += lh;

    snprintf(buf, sizeof(buf), "Queue depth: %d", f->queueDepth);
    DrawText(buf, px, py, 13, C_TEXT); py += lh;

    double throughput = (f->elapsedSec > 0.0)
                        ? (double)f->totalDone / f->elapsedSec : 0.0;
    snprintf(buf, sizeof(buf), "Throughput : %.2f t/s", throughput);
    DrawText(buf, px, py, 13, C_TEXT); py += lh;

    double avgWait = (f->totalDone > 0)
                     ? f->totalWaitTime / f->totalDone : 0.0;
    snprintf(buf, sizeof(buf), "Avg wait   : %.2f s", avgWait);
    DrawText(buf, px, py, 13, C_TEXT); py += lh;

    /* ---- Divider ---- */
    DrawLine(px, py, px + PANEL_W - 10, py, C_BORDER); py += 10;

    /* ---- Synchronization metrics ---- */
    DrawText("SYNC METRICS", px, py, 14, C_ACCENT); py += lh;

    /* Critical Zone semaphore */
    Color czCol = (f->zoneInUse >= 2) ? C_WAIT_ZONE : C_MOVING;
    snprintf(buf, sizeof(buf), "CZ in use  : %d / 2", f->zoneInUse);
    DrawText(buf, px, py, 13, czCol); py += lh;

    snprintf(buf, sizeof(buf), "Sem blocks : %d", f->zoneBlockCount);
    Color sbCol = (f->zoneBlockCount > 0) ? C_WAIT_ZONE : C_TEXT;
    DrawText(buf, px, py, 13, sbCol); py += lh;

    snprintf(buf, sizeof(buf), "Cell waits : %d", f->collisionWaits);
    Color cwCol = (f->collisionWaits > 0) ? C_WAIT_CELL : C_TEXT;
    DrawText(buf, px, py, 13, cwCol); py += lh;

    /* ---- Divider ---- */
    DrawLine(px, py, px + PANEL_W - 10, py, C_BORDER); py += 10;

    /* ---- Per-robot table ---- */
    DrawText("PER-ROBOT STATUS", px, py, 14, C_ACCENT); py += lh;

    /* Header */
    DrawText("ID  STATE       DONE ZW CW", px, py, 11, C_TEXT_DIM); py += 16;

    for (int i = 0; i < f->numRobots; i++) {
        const GuiRobotSnapshot *rb = &f->robots[i];
        Color rc = robot_color(rb->state);

        /* Coloured indicator square */
        DrawRectangle(px, py + 1, 10, 10, rc);

        snprintf(buf, sizeof(buf), "  R%d %-9s  %3d %2d %2d",
                 rb->id,
                 robot_state_name(rb->state),
                 rb->tasksCompleted,
                 rb->zoneWaits,
                 rb->collisionWaits);
        DrawText(buf, px, py, 12, C_TEXT);
        py += 18;
    }

    /* ---- Divider ---- */
    DrawLine(px, py, px + PANEL_W - 10, py, C_BORDER); py += 10;

    /* ---- State legend ---- */
    DrawText("THREAD STATE LEGEND", px, py, 12, C_ACCENT); py += 16;

    struct { Color c; const char *lbl; } legend[] = {
        { C_MOVING,    "MOVING     (running on CPU)"    },
        { C_WAIT_ZONE, "WAIT ZONE  (blocked on sem)"    },
        { C_WAIT_CELL, "WAIT CELL  (blocked on mutex)"  },
        { C_IDLE,      "IDLE       (sleeping/yielded)"  },
    };
    for (int l = 0; l < 4; l++) {
        DrawRectangle(px, py + 1, 10, 10, legend[l].c);
        DrawText(legend[l].lbl, px + 14, py, 11, C_TEXT_DIM);
        py += 15;
    }
}

/* ================================================================
   gui_draw_frame
   Master draw routine.  Called once per frame from the main loop.
   No mutexes held here -- all data comes from the pre-sampled
   GuiFrame, so robot threads are never blocked by rendering.
   ================================================================ */
void gui_draw_frame(const GuiFrame *f, int simRunning) {
    BeginDrawing();
    ClearBackground(C_BG);

    /* ---- Title bar ---- */
    DrawText("SMART WAREHOUSE ROBOT COORDINATOR",
             GRID_OFF_X, 12, 20, C_TITLE);

    /* ---- Main grid (left side) ---- */
    draw_grid(f);

    /* ---- Robot overlays (on top of grid) ---- */
    draw_robots(f);

    /* ---- Stats panel (right side) ---- */
    draw_stats_panel(f, simRunning);

    /* ---- Frame rate (bottom-right debug info) ---- */
    char fpsbuf[32];
    snprintf(fpsbuf, sizeof(fpsbuf), "FPS: %d", GetFPS());
    DrawText(fpsbuf, WIN_W - 80, WIN_H - 20, 12, C_TEXT_DIM);

    EndDrawing();
}
