/* ================================================================
   gui.c  --  Revamped Raylib GUI for Smart Warehouse Simulator

   Visual layout:
     Row 0 (top)    : SHELVES  -- warm brown background, item slots
     Row 1-3 (mid)  : FLOOR    -- dark blue navigation area
     Row 4 (bottom) : DOCKS    -- teal pads where robots wait
     Columns 2-3    : CRITICAL ZONE -- marked orange border

   Thread safety:
     gui_sample_frame() locks briefly to copy state.
     gui_draw_frame()   draws from the private copy -- no locks held.
   ================================================================ */

#include "gui.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>

/* ================================================================
   Colour palette
   ================================================================ */
static const Color C_BG          = {  15,  17,  26, 255 }; /* near-black bg     */
static const Color C_SHELF       = {  80,  52,  20, 255 }; /* warm brown shelf  */
static const Color C_FLOOR       = {  30,  36,  55, 255 }; /* dark blue floor   */
static const Color C_FLOOR_CZ    = {  55,  30,  15, 255 }; /* CZ floor          */
static const Color C_DOCK        = {  18,  55,  55, 255 }; /* teal dock pad     */
static const Color C_OCC         = {  45,  58,  95, 255 }; /* occupied cell     */
static const Color C_ITEM_FLOOR  = { 240, 190,  50, 255 }; /* yellow floor item */
static const Color C_ITEM_SHELF  = { 100, 220, 120, 255 }; /* green shelf item  */
static const Color C_PANEL_BG    = {  22,  25,  40, 255 };
static const Color C_BORDER      = {  65,  75, 110, 255 };
static const Color C_BORDER_CZ   = { 200,  90,  30, 255 };
static const Color C_TEXT        = { 210, 215, 230, 255 };
static const Color C_TEXT_DIM    = { 110, 120, 148, 255 };
static const Color C_ACCENT      = {  80, 160, 255, 255 };
static const Color C_TITLE       = { 255, 200,  80, 255 };
static const Color C_HINT        = {  80, 220, 140, 200 };
static const Color C_PATH_DOT    = { 255, 255, 255,  55 }; /* faint path trail  */
static const Color C_HOVER_FILL  = {  80, 200, 255,  45 };
static const Color C_HOVER_RING  = {  80, 200, 255, 200 };

/* Robot state colours */
static const Color C_MOVING      = {  50, 220,  90, 255 };
static const Color C_WAIT_ZONE   = { 230,  65,  65, 255 };
static const Color C_WAIT_CELL   = { 230, 120,  35, 255 };
static const Color C_IDLE        = { 150, 158, 185, 255 };

/* ================================================================
   gui_init / gui_close
   ================================================================ */
void gui_init(void) {
    SetTraceLogLevel(LOG_WARNING);
    InitWindow(WIN_W, WIN_H, "Smart Warehouse Robot Coordinator");
    SetTargetFPS(FPS);
}
void gui_close(void) { CloseWindow(); }

/* ================================================================
   gui_cell_center
   ================================================================ */
Vector2 gui_cell_center(int gridX, int gridY) {
    return (Vector2){
        (float)(GRID_OFF_X + gridX * (CELL_PX + GAP_PX) + CELL_PX / 2),
        (float)(GRID_OFF_Y + gridY * (CELL_PX + GAP_PX) + CELL_PX / 2)
    };
}

/* ================================================================
   robot_color / robot_state_name
   ================================================================ */
static Color robot_color(RobotState s) {
    switch (s) {
        case ROBOT_MOVING:           return C_MOVING;
        case ROBOT_WAITING_FOR_ZONE: return C_WAIT_ZONE;
        case ROBOT_WAITING_FOR_CELL: return C_WAIT_CELL;
        default:                     return C_IDLE;
    }
}
static const char *robot_state_name(RobotState s) {
    switch (s) {
        case ROBOT_MOVING:           return "MOVING";
        case ROBOT_WAITING_FOR_ZONE: return "WAIT ZONE";
        case ROBOT_WAITING_FOR_CELL: return "WAIT CELL";
        default:                     return "IDLE";
    }
}

/* ================================================================
   gui_sample_frame
   ================================================================ */
void gui_sample_frame(Warehouse *wh, int numRobots, GuiFrame *f,
                      time_t simStart) {
    int i, r, c;

    /* 1. Grid + robot states */
    pthread_mutex_lock(&wh->stateMutex);
    for (r = 0; r < ROWS; r++)
        for (c = 0; c < COLS; c++)
            f->cellOcc[r][c] = wh->cellOccupancy[r][c];
    f->numRobots = (numRobots < MAX_ROBOTS) ? numRobots : MAX_ROBOTS;
    for (i = 0; i < f->numRobots; i++) {
        f->robots[i].id      = i + 1;
        f->robots[i].state   = wh->robotState[i];
        f->robots[i].hasItem = wh->robotHasItem[i];
        f->robots[i].targetX = wh->robotTargetX[i];
        f->robots[i].targetY = wh->robotTargetY[i];
    }
    pthread_mutex_unlock(&wh->stateMutex);

    /* 2. Stats */
    pthread_mutex_lock(&wh->statsMutex);
    f->totalDone      = wh->totalTasksCompleted;
    f->zoneInUse      = wh->zoneInUse;
    f->zoneBlockCount = wh->zoneBlockCount;
    f->collisionWaits = wh->totalCollisionWaits;
    f->totalWaitTime  = wh->totalWaitTime;
    for (i = 0; i < f->numRobots; i++) {
        f->robots[i].tasksCompleted = wh->robotTasksCompleted[i];
        f->robots[i].zoneWaits      = wh->robotZoneWaits[i];
        f->robots[i].collisionWaits = wh->robotCollisionWaits[i];
    }
    pthread_mutex_unlock(&wh->statsMutex);

    /* 3. Items + queue */
    pthread_mutex_lock(&wh->taskMutex);
    f->itemCount = (wh->itemCount < MAX_ITEMS) ? wh->itemCount : MAX_ITEMS;
    for (i = 0; i < f->itemCount; i++) {
        f->itemAvail[i]     = wh->items[i].available;
        f->itemCompleted[i] = wh->items[i].completed;
        f->itemX[i]         = wh->items[i].x;
        f->itemY[i]         = wh->items[i].y;
    }
    f->queueDepth = wh->taskQueue.size;
    pthread_mutex_unlock(&wh->taskMutex);

    f->elapsedSec = difftime(time(NULL), simStart);
}

/* ================================================================
   draw_grid  --  main grid with shelves, floor, docks, CZ, items
   ================================================================ */
static void draw_grid(const GuiFrame *f, int hoverGX, int hoverGY) {
    /* Section label */
    DrawText("WAREHOUSE", GRID_OFF_X, 52, 14, C_ACCENT);

    /* Row labels */
    const char *rowLabels[ROWS] = { "SHELF", "FLOOR", "FLOOR", "FLOOR", "DOCK" };
    for (int r = 0; r < ROWS; r++) {
        int ly = GRID_OFF_Y + r * (CELL_PX + GAP_PX) + CELL_PX / 2 - 6;
        DrawText(rowLabels[r], GRID_OFF_X - 56, ly, 10,
                 r == 0 ? C_ITEM_FLOOR : r == 4 ? C_HINT : C_TEXT_DIM);
    }

    /* Column numbers */
    for (int c = 0; c < COLS; c++) {
        char lbl[4]; snprintf(lbl, 4, "%d", c);
        int lx = GRID_OFF_X + c * (CELL_PX + GAP_PX) + CELL_PX / 2 - 4;
        DrawText(lbl, lx, GRID_OFF_Y - 18, 13, C_TEXT_DIM);
    }

    /* Draw cells */
    for (int r = 0; r < ROWS; r++) {
        for (int c = 0; c < COLS; c++) {
            int px = GRID_OFF_X + c * (CELL_PX + GAP_PX);
            int py = GRID_OFF_Y + r * (CELL_PX + GAP_PX);
            int occ  = f->cellOcc[r][c];
            int inCZ = (c == 2 || c == 3);

            /* Choose background */
            Color bg;
            if (r == 0)       bg = occ != -1 ? C_OCC : C_SHELF;
            else if (r == 4)  bg = occ != -1 ? C_OCC : C_DOCK;
            else if (inCZ)    bg = occ != -1 ? C_OCC : C_FLOOR_CZ;
            else              bg = occ != -1 ? C_OCC : C_FLOOR;

            DrawRectangle(px, py, CELL_PX, CELL_PX, bg);

            /* Shelf row: draw a slim plank line */
            if (r == 0)
                DrawLine(px + 4, py + CELL_PX - 8, px + CELL_PX - 4,
                         py + CELL_PX - 8, (Color){180, 130, 60, 80});

            /* Dock row: draw pad circle */
            if (r == 4 && occ == -1)
                DrawCircleLines(px + CELL_PX/2, py + CELL_PX/2,
                                14, (Color){40, 160, 160, 80});

            /* CZ border (orange) */
            if (inCZ && r > 0 && r < 4)
                DrawRectangleLines(px, py, CELL_PX, CELL_PX, C_BORDER_CZ);
            else if (c == hoverGX && r == hoverGY) {
                DrawRectangle(px, py, CELL_PX, CELL_PX, C_HOVER_FILL);
                DrawRectangleLines(px, py, CELL_PX, CELL_PX, C_HOVER_RING);
                DrawRectangleLines(px+1, py+1, CELL_PX-2, CELL_PX-2, C_HOVER_RING);
            } else {
                DrawRectangleLines(px, py, CELL_PX, CELL_PX, C_BORDER);
            }

            /* Row/Col coord label */
            char coord[8]; snprintf(coord, 8, "%d,%d", c, r);
            DrawText(coord, px + 4, py + 4, 9, C_TEXT_DIM);

            /* CZ label on floor cells */
            if (inCZ && r >= 1 && r <= 3 && occ == -1)
                DrawText("CZ", px + CELL_PX/2 - 8, py + CELL_PX/2 - 6,
                         12, (Color){200, 100, 50, 100});

            /* SHELF label on empty shelf cells */
            if (r == 0 && occ == -1) {
                DrawText("SHELF", px + 4, py + CELL_PX/2 - 6, 10,
                         (Color){200, 160, 80, 90});
            }

            /* DOCK label */
            if (r == 4 && occ == -1)
                DrawText("DOCK", px + CELL_PX/2 - 14, py + CELL_PX - 18,
                         10, (Color){60, 180, 180, 80});
        }
    }

    /* Draw delivered items on shelves (green) */
    for (int i = 0; i < f->itemCount; i++) {
        if (!f->itemCompleted[i]) continue;
        int ix = f->itemX[i], iy = f->itemY[i];
        if (iy != 0 || ix < 0 || ix >= COLS) continue;
        Vector2 ctr = gui_cell_center(ix, iy);
        DrawRectangle((int)ctr.x - 10, (int)ctr.y - 10, 20, 20, C_ITEM_SHELF);
        DrawRectangleLines((int)ctr.x - 10, (int)ctr.y - 10, 20, 20,
                           (Color){160, 255, 180, 180});
        DrawText("pkg", (int)ctr.x - 9, (int)ctr.y - 6, 10, C_BG);
    }

    /* Draw floor items (yellow circles) */
    for (int i = 0; i < f->itemCount; i++) {
        if (!f->itemAvail[i]) continue;
        int ix = f->itemX[i], iy = f->itemY[i];
        if (ix < 0 || ix >= COLS || iy < 0 || iy >= ROWS) continue;
        Vector2 ctr = gui_cell_center(ix, iy);
        DrawCircleV(ctr, 9.0f, C_ITEM_FLOOR);
        DrawCircleLines((int)ctr.x, (int)ctr.y, 9,
                        (Color){255, 240, 120, 200});
        DrawText("!", (int)ctr.x - 2, (int)ctr.y - 6, 12, C_BG);
    }

    /* Interaction hint */
    int hintY = GRID_OFF_Y + ROWS * (CELL_PX + GAP_PX) + 14;
    float pulse = (float)(0.55 + 0.45 * sin(GetTime() * 2.2));
    Color hintCol = {C_HINT.r, C_HINT.g, C_HINT.b, (unsigned char)(pulse * 210)};
    DrawText("[ Click FLOOR rows 1-3 to spawn a delivery task ]",
             GRID_OFF_X, hintY, 13, hintCol);

    /* Legend row */
    int legY = hintY + 18;
    DrawRectangle(GRID_OFF_X,      legY, 12, 10, C_SHELF);
    DrawText("= Shelf",  GRID_OFF_X + 15,  legY, 11, C_TEXT_DIM);
    DrawRectangle(GRID_OFF_X + 75, legY, 12, 10, C_DOCK);
    DrawText("= Dock",   GRID_OFF_X + 90,  legY, 11, C_TEXT_DIM);
    DrawRectangle(GRID_OFF_X + 148, legY, 12, 10, C_FLOOR_CZ);
    DrawText("= CZ (sem)", GRID_OFF_X + 163, legY, 11, C_TEXT_DIM);
    DrawCircle(GRID_OFF_X + 252, legY + 5, 5, C_ITEM_FLOOR);
    DrawText("= Item", GRID_OFF_X + 260,  legY, 11, C_TEXT_DIM);
    DrawRectangle(GRID_OFF_X + 308, legY, 10, 10, C_ITEM_SHELF);
    DrawText("= Shelved", GRID_OFF_X + 322, legY, 11, C_TEXT_DIM);
}

/* ================================================================
   draw_robots  --  circles + path lines to target
   ================================================================ */
static void draw_robots(const GuiFrame *f) {
    /* Draw path lines FIRST (behind robots) */
    for (int i = 0; i < f->numRobots; i++) {
        const GuiRobotSnapshot *rb = &f->robots[i];
        if (rb->targetX < 0 || rb->targetY < 0) continue;

        /* Find robot's screen position from cell occupancy */
        int foundR = -1, foundC = -1;
        for (int r = 0; r < ROWS && foundR == -1; r++)
            for (int c = 0; c < COLS && foundR == -1; c++)
                if (f->cellOcc[r][c] == rb->id) { foundR = r; foundC = c; }

        if (foundR == -1) continue;

        Vector2 from = gui_cell_center(foundC, foundR);
        Vector2 to   = gui_cell_center(rb->targetX, rb->targetY);

        /* Draw dotted line from robot to target */
        float dx = to.x - from.x, dy = to.y - from.y;
        float dist = sqrtf(dx*dx + dy*dy);
        int steps = (int)(dist / 18.0f);
        for (int s = 1; s < steps; s++) {
            float t = (float)s / steps;
            float sx = from.x + dx * t;
            float sy = from.y + dy * t;
            DrawCircle((int)sx, (int)sy, 2.5f, C_PATH_DOT);
        }
        /* Target marker (X at destination) */
        DrawCircleLines((int)to.x, (int)to.y, 10,
                        (Color){255, 255, 255, 60});
    }

    /* Draw robots */
    for (int i = 0; i < f->numRobots; i++) {
        const GuiRobotSnapshot *rb = &f->robots[i];

        int foundR = -1, foundC = -1;
        for (int r = 0; r < ROWS && foundR == -1; r++)
            for (int c = 0; c < COLS && foundR == -1; c++)
                if (f->cellOcc[r][c] == rb->id) { foundR = r; foundC = c; }

        float px, py;
        if (foundR != -1) {
            Vector2 cv = gui_cell_center(foundC, foundR);
            px = cv.x; py = cv.y;
        } else {
            px = (float)(GRID_OFF_X + i * 90 + CELL_PX/2);
            py = (float)(GRID_OFF_Y + (ROWS + 1) * (CELL_PX + GAP_PX));
        }

        Color col = robot_color(rb->state);

        /* Glow ring when blocked */
        if (rb->state == ROBOT_WAITING_FOR_ZONE ||
            rb->state == ROBOT_WAITING_FOR_CELL) {
            DrawCircleLines((int)px, (int)py, ROBOT_R + 5,
                            (Color){col.r, col.g, col.b, 90});
        }

        DrawCircleV((Vector2){px, py}, (float)ROBOT_R, col);
        DrawCircleLines((int)px, (int)py, ROBOT_R, WHITE);

        char idstr[4]; snprintf(idstr, 4, "R%d", rb->id);
        int tw = MeasureText(idstr, 12);
        DrawText(idstr, (int)px - tw/2, (int)py - 6, 12, BLACK);

        /* Item carried dot */
        if (rb->hasItem) {
            DrawCircle((int)px + 7, (int)py + 7, 5, C_ITEM_FLOOR);
            DrawCircleLines((int)px + 7, (int)py + 7, 5, C_BG);
        }
    }
}

/* ================================================================
   draw_stats_panel  --  right-hand live monitor
   ================================================================ */
static void draw_stats_panel(const GuiFrame *f, int simRunning) {
    int px = PANEL_X, py = 40, lh = 22;

    DrawRectangle(px - 10, 30, PANEL_W + 10, WIN_H - 40, C_PANEL_BG);
    DrawRectangleLines(px - 10, 30, PANEL_W + 10, WIN_H - 40, C_BORDER);

    DrawText("LIVE MONITOR", px, py, 18, C_TITLE); py += 28;

    Color sc = simRunning ? C_MOVING : C_WAIT_ZONE;
    DrawText(simRunning ? "[ RUNNING ]" : "[ STOPPED ]", px, py, 15, sc);
    py += lh + 4;

    char buf[128];
    snprintf(buf, sizeof(buf), "Elapsed : %.0f s", f->elapsedSec);
    DrawText(buf, px, py, 14, C_TEXT); py += lh;

    DrawLine(px, py, px + PANEL_W - 10, py, C_BORDER); py += 10;
    DrawText("TASK SCHEDULER", px, py, 14, C_ACCENT); py += lh;

    snprintf(buf, sizeof(buf), "Completed : %d", f->totalDone);
    DrawText(buf, px, py, 13, C_TEXT); py += lh;

    snprintf(buf, sizeof(buf), "In queue  : %d", f->queueDepth);
    DrawText(buf, px, py, 13, C_TEXT); py += lh;

    snprintf(buf, sizeof(buf), "Items     : %d", f->itemCount);
    DrawText(buf, px, py, 13, C_TEXT); py += lh;

    double tp = f->elapsedSec > 0 ? (double)f->totalDone / f->elapsedSec : 0;
    snprintf(buf, sizeof(buf), "Throughput: %.2f t/s", tp);
    DrawText(buf, px, py, 13, C_TEXT); py += lh;

    DrawLine(px, py, px + PANEL_W - 10, py, C_BORDER); py += 10;
    DrawText("SYNC METRICS", px, py, 14, C_ACCENT); py += lh;

    Color czC = (f->zoneInUse >= 2) ? C_WAIT_ZONE : C_MOVING;
    snprintf(buf, sizeof(buf), "CZ in use : %d / 2", f->zoneInUse);
    DrawText(buf, px, py, 13, czC); py += lh;

    Color sbC = f->zoneBlockCount > 0 ? C_WAIT_ZONE : C_TEXT;
    snprintf(buf, sizeof(buf), "Sem blocks: %d", f->zoneBlockCount);
    DrawText(buf, px, py, 13, sbC); py += lh;

    Color cwC = f->collisionWaits > 0 ? C_WAIT_CELL : C_TEXT;
    snprintf(buf, sizeof(buf), "Cell waits: %d", f->collisionWaits);
    DrawText(buf, px, py, 13, cwC); py += lh;

    DrawLine(px, py, px + PANEL_W - 10, py, C_BORDER); py += 10;
    DrawText("PER-ROBOT STATUS", px, py, 14, C_ACCENT); py += lh;
    DrawText("    STATE       DONE ZW CW", px, py, 11, C_TEXT_DIM); py += 16;

    for (int i = 0; i < f->numRobots; i++) {
        const GuiRobotSnapshot *rb = &f->robots[i];
        Color rc = robot_color(rb->state);
        DrawRectangle(px, py + 1, 10, 10, rc);
        if (rb->hasItem)
            DrawCircle(px + 17, py + 5, 4, C_ITEM_FLOOR);
        snprintf(buf, sizeof(buf), "  R%d %-9s %3d %2d %2d",
                 rb->id, robot_state_name(rb->state),
                 rb->tasksCompleted, rb->zoneWaits, rb->collisionWaits);
        DrawText(buf, px, py, 12, C_TEXT);
        py += 18;
    }

    DrawLine(px, py, px + PANEL_W - 10, py, C_BORDER); py += 10;
    DrawText("THREAD STATE LEGEND", px, py, 12, C_ACCENT); py += 16;
    struct { Color c; const char *lbl; } leg[] = {
        {C_MOVING,    "MOVING     (on CPU)"},
        {C_WAIT_ZONE, "WAIT ZONE  (sem blocked)"},
        {C_WAIT_CELL, "WAIT CELL  (mutex blocked)"},
        {C_IDLE,      "IDLE       (at dock)"},
    };
    for (int l = 0; l < 4; l++) {
        DrawRectangle(px, py + 1, 10, 10, leg[l].c);
        DrawText(leg[l].lbl, px + 14, py, 11, C_TEXT_DIM);
        py += 15;
    }

    DrawLine(px, py, px + PANEL_W - 10, py, C_BORDER); py += 10;
    DrawText("CONTROLS", px, py, 12, C_ACCENT); py += 16;
    DrawText("LClick floor row 1-3  spawn task", px, py, 11, C_HINT); py += 14;
    DrawText("Esc / X               quit",        px, py, 11, C_TEXT_DIM);
}

/* ================================================================
   gui_draw_frame
   ================================================================ */
void gui_draw_frame(const GuiFrame *f, int simRunning,
                    int hoverGX, int hoverGY) {
    BeginDrawing();
    ClearBackground(C_BG);

    DrawText("SMART WAREHOUSE ROBOT COORDINATOR",
             GRID_OFF_X, 12, 20, C_TITLE);

    draw_grid(f, hoverGX, hoverGY);
    draw_robots(f);
    draw_stats_panel(f, simRunning);

    char fps[32]; snprintf(fps, 32, "FPS: %d", GetFPS());
    DrawText(fps, WIN_W - 80, WIN_H - 20, 12, C_TEXT_DIM);

    EndDrawing();
}
