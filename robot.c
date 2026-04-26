/* ================================================================
   robot.c  --  Step-by-step robot navigation engine

   Coordinate system:
     x = column  (0 = left,  COLS-1 = right)
     y = row     (0 = top/shelves, ROWS-1 = bottom/docks)
   cellOccupancy[y][x] and gridMutex[y][x]  (row first, col second)

   Task flow per robot:
     IDLE at dock  ->  go to pickup (floor, y=1..3)
               ->  pick up item
               ->  go to drop (shelf, y=0)
               ->  drop item
               ->  return to dock (y=4)
               ->  repeat

   Deadlock/Livelock resolution strategy:
     1. Try the direct (greedy) next cell.
     2. If blocked, try ALL valid adjacent cells in random order.
     3. If ALL adjacent cells are blocked and failStreak is high,
        the LOWER-priority robot (higher ID) backs off: it yields
        its CPU slice and waits so the other robot can break free.
        The HIGHER-priority robot (lower ID) waits only a short fixed
        amount. This asymmetry guarantees one robot always makes
        progress, resolving both livelock and deadlock.
   ================================================================ */

#define _DEFAULT_SOURCE
#include "robot.h"
#include <unistd.h>
#include <stdlib.h>
#include <sched.h>
#include <time.h>
#include <stdio.h>
#include <string.h>

#define STEP_DELAY_US    1200000  /* 1.2 s per cell step                  */
#define LOCK_TIMEOUT_MS  100      /* cell mutex timeout                    */
#define BASE_WAIT_US     300000   /* 300ms base retry interval             */
#define BACKTRACK_AFTER  3        /* failStreak threshold to trigger backtrack */
#define CZ_COL_MIN       2        /* Critical Zone: columns 2 and 3        */
#define CZ_COL_MAX       3

/* ---------------------------------------------------------------- */
static int lockCellTimed(pthread_mutex_t *m, int ms) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec  += ms / 1000;
    ts.tv_nsec += (ms % 1000) * 1000000L;
    if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }
    return pthread_mutex_timedlock(m, &ts) == 0;
}

static void applyAging(Warehouse *wh) {
    pthread_mutex_lock(&wh->taskMutex);
    for (int i = 0; i < wh->taskQueue.size; i++)
        if (wh->taskQueue.arr[i].priority < 10)
            wh->taskQueue.arr[i].priority++;
    pthread_mutex_unlock(&wh->taskMutex);
}

/* ----------------------------------------------------------------
   setTarget  --  update robotTargetX/Y for GUI path display
   ---------------------------------------------------------------- */
static void setTarget(Warehouse *wh, int robotId, int tx, int ty) {
    pthread_mutex_lock(&wh->stateMutex);
    wh->robotTargetX[robotId - 1] = tx;
    wh->robotTargetY[robotId - 1] = ty;
    pthread_mutex_unlock(&wh->stateMutex);
}

/* ----------------------------------------------------------------
   shuffle4  --  Fisher-Yates shuffle for 4-element array
   ---------------------------------------------------------------- */
static void shuffle4(int arr[][2], int n) {
    for (int i = n - 1; i > 0; i--) {
        int j = rand() % (i + 1);
        int tx = arr[i][0], ty = arr[i][1];
        arr[i][0] = arr[j][0]; arr[i][1] = arr[j][1];
        arr[j][0] = tx;        arr[j][1] = ty;
    }
}

/* ----------------------------------------------------------------
   commitMove  --  atomically move robot from current cell to (nx,ny)
                   Caller must already hold gridMutex[ny][nx].
   ---------------------------------------------------------------- */
static void commitMove(Warehouse *wh, Robot *r, int nx, int ny,
                       int curCZ, int nextCZ) {
    setRobotState(wh, r->id, ROBOT_MOVING);

    pthread_mutex_lock(&wh->stateMutex);
    wh->cellOccupancy[ny][nx]         = r->id;
    pthread_mutex_unlock(&wh->stateMutex);

    usleep(STEP_DELAY_US);

    pthread_mutex_lock(&wh->stateMutex);
    wh->cellOccupancy[r->curY][r->curX] = -1;
    pthread_mutex_unlock(&wh->stateMutex);
    pthread_mutex_unlock(&wh->gridMutex[r->curY][r->curX]);

    /* Release CZ semaphore when leaving the zone */
    if (curCZ && !nextCZ) {
        pthread_mutex_lock(&wh->statsMutex);
        wh->zoneInUse--;
        pthread_mutex_unlock(&wh->statsMutex);
        sem_post(&wh->zoneSemaphore);
    }

    r->curX = nx;
    r->curY = ny;
}

/* ----------------------------------------------------------------
   tryAcquireCell  --  attempt to lock a candidate cell.
   Returns 1 and sets *outCZ if successful, 0 otherwise.
   Handles CZ semaphore acquisition/release internally.
   ---------------------------------------------------------------- */
static int tryAcquireCell(Warehouse *wh, Robot *r,
                          int cx, int cy, int *outCZ) {
    int curCZ  = (r->curX >= CZ_COL_MIN && r->curX <= CZ_COL_MAX);
    int candCZ = (cx >= CZ_COL_MIN && cx <= CZ_COL_MAX);

    /* Entering CZ from outside -- need semaphore */
    if (!curCZ && candCZ) {
        if (sem_trywait(&wh->zoneSemaphore) != 0)
            return 0;   /* CZ full, skip this candidate */
        pthread_mutex_lock(&wh->statsMutex);
        wh->zoneInUse++;
        wh->zoneUsageEvents++;
        pthread_mutex_unlock(&wh->statsMutex);
    }

    if (lockCellTimed(&wh->gridMutex[cy][cx], LOCK_TIMEOUT_MS)) {
        *outCZ = candCZ;
        return 1;
    }

    /* Lock failed -- release CZ token we just took */
    if (!curCZ && candCZ) {
        pthread_mutex_lock(&wh->statsMutex);
        wh->zoneInUse--;
        pthread_mutex_unlock(&wh->statsMutex);
        sem_post(&wh->zoneSemaphore);
    }
    return 0;
}

/* ----------------------------------------------------------------
   navigate_to  --  move robot one cell at a time to (targetX, targetY)

   Invariant: robot always holds gridMutex[curY][curX].
   ---------------------------------------------------------------- */
static void navigate_to(Warehouse *wh, Robot *r, int targetX, int targetY) {
    setTarget(wh, r->id, targetX, targetY);

    int failStreak = 0;

    while (wh->running && (r->curX != targetX || r->curY != targetY)) {

        /* ---- Step 1: Build the preferred (greedy) next cell ---- */
        int prefX = r->curX, prefY = r->curY;
        if (r->curX != targetX)
            prefX += (r->curX < targetX) ? 1 : -1;
        else
            prefY += (r->curY < targetY) ? 1 : -1;

        /* ---- Step 2: Build ALL 4 neighbours, greedy direction first ---- */
        /* Preferred direction is [0]; the other 3 alternatives are shuffled
           randomly so that tie-breaking is different each attempt. */
        int other[3][2] = {
            { r->curX + 1, r->curY    },
            { r->curX - 1, r->curY    },
            { r->curX,     r->curY + 1},
        };
        /* If preferred is horizontal replace one horizontal with vertical */
        if (prefX != r->curX) {
            other[0][0] = r->curX;     other[0][1] = r->curY + 1;
            other[1][0] = r->curX;     other[1][1] = r->curY - 1;
            other[2][0] = r->curX + (prefX > r->curX ? -1 : 1);
            other[2][1] = r->curY;
        } else {
            other[0][0] = r->curX + 1; other[0][1] = r->curY;
            other[1][0] = r->curX - 1; other[1][1] = r->curY;
            other[2][0] = r->curX;     other[2][1] = r->curY + (prefY > r->curY ? -1 : 1);
        }
        /* Shuffle the 3 alternatives so both robots don't always pick same dodge */
        shuffle4(other, 3);

        /* Final ordered candidate list: [preferred, alt1, alt2, alt3] */
        int ordered[4][2] = {
            { prefX,       prefY       },
            { other[0][0], other[0][1] },
            { other[1][0], other[1][1] },
            { other[2][0], other[2][1] }
        };

        /* ---- Step 3: Backtrack mode when severely stuck ----
           Lower-priority robot (higher id) backs UP (away from target)
           to physically create space. Higher-priority robot (lower id)
           waits a fixed short interval so the lower-priority one can move. */
        int backtracking = 0;
        if (failStreak >= BACKTRACK_AFTER) {
            /* Lower-priority robot backs up */
            if (r->id > 1) {   /* robot IDs: 1 = highest priority */
                int bx = r->curX, by = r->curY;
                if (r->curX != targetX)
                    bx -= (r->curX < targetX) ? 1 : -1;  /* step back */
                else
                    by -= (r->curY < targetY) ? 1 : -1;
                /* Override ordered list: backtrack cell first */
                ordered[0][0] = bx; ordered[0][1] = by;
                backtracking = 1;
                safeLog(wh, r->id, "[backtrack] yielding space to higher-priority robot");
            } else {
                /* Highest-priority robot just waits briefly */
                safeLog(wh, r->id, "[yield] waiting for lower-priority robot to clear");
                sched_yield();
                usleep(BASE_WAIT_US);
                continue;
            }
        }

        /* ---- Step 4: Try each candidate in order ---- */
        setRobotState(wh, r->id, ROBOT_WAITING_FOR_CELL);
        int moved = 0;

        for (int i = 0; i < 4 && !moved; i++) {
            int cx = ordered[i][0], cy = ordered[i][1];

            /* Bounds: stay within grid, avoid row 0 (shelves) for dodges
               unless that IS the actual target row, avoid going backward
               into dock row unless returning home. */
            if (cx < 0 || cx >= COLS || cy < 0 || cy >= ROWS) continue;
            if (cy == 0 && !(targetY == 0)) continue;   /* don't trespass shelves */
            if (cy == ROWS-1 && !(targetY == ROWS-1) && i > 0) continue; /* avoid docks when not needed */

            /* CZ guard: don't enter CZ as a mere dodge if not heading there */
            int candCZ = (cx >= CZ_COL_MIN && cx <= CZ_COL_MAX);
            int curCZ  = (r->curX >= CZ_COL_MIN && r->curX <= CZ_COL_MAX);
            if (!curCZ && candCZ && i > 0) {
                /* Dodge into CZ only if we can get the semaphore right now */
                /* (tryAcquireCell will handle this; skip if no token) */
            }

            int gotCZ = 0;
            if (tryAcquireCell(wh, r, cx, cy, &gotCZ)) {
                if (i > 0 && !backtracking)
                    safeLog(wh, r->id, "[dodge] stepped aside");
                else if (backtracking)
                    safeLog(wh, r->id, "[backtrack] moved back");

                int nextCZ_flag = (cx >= CZ_COL_MIN && cx <= CZ_COL_MAX);
                int wasCZ       = (r->curX >= CZ_COL_MIN && r->curX <= CZ_COL_MAX);

                /* If entering CZ via preferred path, record zone entry stats */
                if (!wasCZ && nextCZ_flag && i == 0 && !gotCZ) {
                    /* preferred path acquired sem in tryAcquireCell already */
                }

                /* Handle CZ semaphore for preferred-path CZ entry that went
                   through sem_wait (blocking) path */
                if (!wasCZ && nextCZ_flag && i == 0) {
                    /* sem already acquired in tryAcquireCell */
                    pthread_mutex_lock(&wh->statsMutex);
                    wh->zoneUsageEvents++;
                    pthread_mutex_unlock(&wh->statsMutex);
                }

                commitMove(wh, r, cx, cy, wasCZ, nextCZ_flag);
                failStreak = 0;
                moved = 1;
                safeLog(wh, r->id, "[step] moved to new cell");
            }
        }

        if (!moved) {
            /* Truly stuck -- nothing is available.
               Yield and sleep with a fixed short pause.
               We do NOT grow the backoff indefinitely to keep robots responsive. */
            pthread_mutex_lock(&wh->statsMutex);
            wh->totalCollisionWaits++;
            wh->robotCollisionWaits[r->id - 1]++;
            pthread_mutex_unlock(&wh->statsMutex);

            failStreak++;
            safeLog(wh, r->id, "[wait] all paths blocked, yielding");
            sched_yield();
            usleep(BASE_WAIT_US);
        }
    }
}

/* ================================================================
   robotFunc  --  main thread loop
   ================================================================ */
void* robotFunc(void *arg) {
    Robot    *r   = (Robot *)arg;
    Warehouse *wh = r->wh;
    int rIdx      = r->id - 1;
    int loggedIdle = 0;

    while (wh->running) {
        Task t;
        setRobotState(wh, r->id, ROBOT_IDLE);
        applyAging(wh);

        if (!popTaskAndClaimItem(wh, &t)) {
            if (!loggedIdle) {
                safeLog(wh, r->id, "[idle] no task -- returning to dock");
                loggedIdle = 1;
            }
            /* Return to dock if not already there */
            if (r->curX != r->dockX || r->curY != r->dockY)
                navigate_to(wh, r, r->dockX, r->dockY);
            setTarget(wh, r->id, r->dockX, r->dockY);
            usleep(400000);
            sched_yield();
            continue;
        }
        loggedIdle = 0;
        safeLog(wh, r->id, "[task_claimed] going to pickup");

        /* --- Phase 1: Go to pickup (floor cell) --- */
        navigate_to(wh, r, t.pickupX, t.pickupY);
        if (!wh->running) break;

        /* Pick up item -- hide from floor */
        pthread_mutex_lock(&wh->taskMutex);
        wh->items[t.itemId].available = 0;
        pthread_mutex_unlock(&wh->taskMutex);
        r->hasItem = 1;
        pthread_mutex_lock(&wh->stateMutex);
        wh->robotHasItem[rIdx] = 1;
        pthread_mutex_unlock(&wh->stateMutex);
        safeLog(wh, r->id, "[pickup] item acquired from floor");

        /* --- Phase 2: Deliver to shelf (y=0) --- */
        navigate_to(wh, r, t.dropX, t.dropY);
        if (!wh->running) break;

        /* Place item on shelf */
        completeItemForTask(wh, &t);
        r->hasItem = 0;
        pthread_mutex_lock(&wh->stateMutex);
        wh->robotHasItem[rIdx] = 0;
        pthread_mutex_unlock(&wh->stateMutex);
        safeLog(wh, r->id, "[dropoff] item placed on shelf");

        /* Record stats */
        long now = time(NULL);
        double waited = difftime(now, t.enqueueTime);
        if (waited < 0) waited = 0;
        pthread_mutex_lock(&wh->statsMutex);
        wh->totalTasksCompleted++;
        wh->totalWaitTime += waited;
        wh->robotTasksCompleted[rIdx]++;
        pthread_mutex_unlock(&wh->statsMutex);

        /* --- Phase 3: Return to dock --- */
        navigate_to(wh, r, r->dockX, r->dockY);
        setTarget(wh, r->id, r->dockX, r->dockY);
        safeLog(wh, r->id, "[docked] back at home position");
        sched_yield();
    }

    /* Cleanup: unlock held cell, release CZ if needed */
    pthread_mutex_unlock(&wh->gridMutex[r->curY][r->curX]);
    if (r->curX >= CZ_COL_MIN && r->curX <= CZ_COL_MAX) {
        pthread_mutex_lock(&wh->statsMutex);
        if (wh->zoneInUse > 0) wh->zoneInUse--;
        pthread_mutex_unlock(&wh->statsMutex);
        sem_post(&wh->zoneSemaphore);
    }
    return NULL;
}
