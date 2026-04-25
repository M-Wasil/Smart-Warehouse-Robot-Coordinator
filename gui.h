#ifndef GUI_H
#define GUI_H

/* ================================================================
   gui.h  --  Raylib GUI declarations for the Warehouse Simulator

   OS / Threading note for viva:
     The GUI runs entirely on the MAIN thread.
     Robot worker threads (pthreads) mutate the Warehouse struct
     concurrently.  The GUI only READS shared state -- it never
     calls pushTask, popTask, or any function that mutates the
     Warehouse -- so it holds locks for the shortest possible time
     (a single pthread_mutex_lock/unlock pair per frame for each
     piece of state it samples).

     This is the standard "producer-consumer with read-only
     observer" pattern:
       - Robots  = producers  (write state under locks)
       - GUI     = observer   (reads state under same locks,
                               then renders without holding any lock)
   ================================================================ */

#include "raylib.h"
#include "warehouse.h"

/* ---- Window dimensions ----------------------------------------- */
#define WIN_W   1000
#define WIN_H   700
#define FPS      60

/* ---- Grid rendering -------------------------------------------- */
/*
 * The 5x5 warehouse grid is drawn in the top-left area.
 * Each cell is CELL_PX x CELL_PX pixels with GAP_PX spacing.
 */
#define CELL_PX   80          /* pixels per grid cell              */
#define GAP_PX     4          /* gap between cells                 */
#define GRID_OFF_X 30         /* x offset of grid from window edge */
#define GRID_OFF_Y 80         /* y offset (leave room for title)   */

/* ---- Panel dimensions ------------------------------------------ */
#define PANEL_X   (GRID_OFF_X + COLS*(CELL_PX+GAP_PX) + 20)
#define PANEL_W   (WIN_W - PANEL_X - 20)

/* ---- Robot circle radius --------------------------------------- */
#define ROBOT_R   16

/* ----------------------------------------------------------------
   GuiRobotSnapshot
   A copy of per-robot data sampled under a mutex each frame.
   The GUI renders from this snapshot -- it never touches the
   live Warehouse fields while drawing (no lock held during draw).
   ---------------------------------------------------------------- */
typedef struct {
    int        id;
    RobotState state;
    int        tasksCompleted;
    int        zoneWaits;
    int        collisionWaits;
    /* Pixel position of the robot on screen (derived from task coords) */
    float      screenX;
    float      screenY;
} GuiRobotSnapshot;

/* ----------------------------------------------------------------
   GuiFrame
   Everything the GUI needs to draw one frame, sampled once per
   frame under the minimum required locks.
   ---------------------------------------------------------------- */
typedef struct {
    /* Grid occupancy snapshot (from cellOccupancy) */
    int cellOcc[ROWS][COLS];      /* -1=empty, else robotId        */

    /* Item snapshot */
    int itemAvail[MAX_ITEMS];     /* 1=item on floor, 0=picked/done*/
    int itemX[MAX_ITEMS];
    int itemY[MAX_ITEMS];
    int itemCount;

    /* Per-robot snapshots */
    GuiRobotSnapshot robots[MAX_ROBOTS];
    int numRobots;

    /* Stats snapshot */
    int    totalDone;
    int    queueDepth;
    int    zoneInUse;
    int    zoneBlockCount;
    int    collisionWaits;
    double totalWaitTime;
    double elapsedSec;
} GuiFrame;

/* ---- Function prototypes --------------------------------------- */

/* One-time setup: open window, set target FPS */
void gui_init(void);

/* Sample a GuiFrame from the warehouse under locks.
   Call this once per render loop iteration BEFORE drawing.
   Holds each mutex only long enough to copy the data out.    */
void gui_sample_frame(Warehouse *wh, int numRobots, GuiFrame *f,
                      time_t simStart);

/* Draw one complete frame from a pre-sampled GuiFrame.
   No mutexes held during this call.                          */
void gui_draw_frame(const GuiFrame *f, int simRunning);

/* Compute pixel centre of grid cell (col=x, row=y) */
Vector2 gui_cell_center(int gridX, int gridY);

/* Close the Raylib window */
void gui_close(void);

#endif /* GUI_H */
