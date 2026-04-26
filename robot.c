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
   ================================================================ */

#define _DEFAULT_SOURCE
#include "robot.h"
#include <unistd.h>
#include <stdlib.h>
#include <sched.h>
#include <time.h>
#include <stdio.h>

#define STEP_DELAY_US   1200000   /* 1.2 s per cell step            */
#define LOCK_TIMEOUT_MS 180       /* cell mutex timeout             */
#define RETRY_WAIT_US   350000    /* wait before retrying a blocked step */
#define CZ_COL_MIN      2         /* Critical Zone: columns 2 and 3 */
#define CZ_COL_MAX      3

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
   navigate_to  --  move robot one cell at a time to (targetX, targetY)

   Invariant: robot always holds gridMutex[curY][curX].
   ---------------------------------------------------------------- */
static void navigate_to(Warehouse *wh, Robot *r, int targetX, int targetY) {
    setTarget(wh, r->id, targetX, targetY);

    while (wh->running && (r->curX != targetX || r->curY != targetY)) {

        /* ---- Decide next cell (horizontal-first greedy) ---- */
        int nx = r->curX, ny = r->curY;
        if (r->curX != targetX)
            nx += (r->curX < targetX) ? 1 : -1;
        else
            ny += (r->curY < targetY) ? 1 : -1;

        int curCZ  = (r->curX >= CZ_COL_MIN && r->curX <= CZ_COL_MAX);
        int nextCZ = (nx >= CZ_COL_MIN && nx <= CZ_COL_MAX);

        /* ---- Acquire CZ semaphore before entering CZ ---- */
        if (!curCZ && nextCZ) {
            setRobotState(wh, r->id, ROBOT_WAITING_FOR_ZONE);
            if (sem_trywait(&wh->zoneSemaphore) != 0) {
                pthread_mutex_lock(&wh->statsMutex);
                wh->zoneBlockCount++;
                wh->robotZoneWaits[r->id - 1]++;
                pthread_mutex_unlock(&wh->statsMutex);
                safeLog(wh, r->id, "[zone_wait] blocked at CZ border");
                sem_wait(&wh->zoneSemaphore);
            }
            pthread_mutex_lock(&wh->statsMutex);
            wh->zoneInUse++;
            wh->zoneUsageEvents++;
            pthread_mutex_unlock(&wh->statsMutex);
        }

        /* ---- Try to lock the next cell ---- */
        setRobotState(wh, r->id, ROBOT_WAITING_FOR_CELL);
        if (!lockCellTimed(&wh->gridMutex[ny][nx], LOCK_TIMEOUT_MS)) {

            /* Undo CZ entry we just acquired */
            if (!curCZ && nextCZ) {
                pthread_mutex_lock(&wh->statsMutex);
                wh->zoneInUse--;
                pthread_mutex_unlock(&wh->statsMutex);
                sem_post(&wh->zoneSemaphore);
            }

            pthread_mutex_lock(&wh->statsMutex);
            wh->totalCollisionWaits++;
            wh->robotCollisionWaits[r->id - 1]++;
            pthread_mutex_unlock(&wh->statsMutex);

            /* Try a perpendicular dodge cell */
            int ax = r->curX, ay = r->curY;
            if (nx != r->curX) {           /* was going horizontal -> try vertical */
                if (r->curY + 1 < ROWS && r->curY + 1 != 0) ay = r->curY + 1;
                else if (r->curY - 1 >= 0 && r->curY - 1 != 0) ay = r->curY - 1;
            } else {                       /* was going vertical -> try horizontal */
                if (r->curX + 1 < COLS) ax = r->curX + 1;
                else if (r->curX - 1 >= 0) ax = r->curX - 1;
            }

            int altCZ = (ax >= CZ_COL_MIN && ax <= CZ_COL_MAX);
            /* Don't dodge into CZ without semaphore */
            if (!curCZ && altCZ) {
                safeLog(wh, r->id, "[collision_wait] blocked, retrying");
                usleep(RETRY_WAIT_US);
                continue;
            }

            if ((ax != r->curX || ay != r->curY) &&
                lockCellTimed(&wh->gridMutex[ay][ax], LOCK_TIMEOUT_MS)) {
                nx = ax; ny = ay; nextCZ = altCZ;
                safeLog(wh, r->id, "[dodge] stepped aside");
            } else {
                safeLog(wh, r->id, "[collision_wait] path blocked, waiting");
                usleep(RETRY_WAIT_US);
                continue;
            }
        }

        /* ---- Commit the move ---- */
        setRobotState(wh, r->id, ROBOT_MOVING);

        /* Mark new cell occupied in GUI */
        pthread_mutex_lock(&wh->stateMutex);
        wh->cellOccupancy[ny][nx] = r->id;
        pthread_mutex_unlock(&wh->stateMutex);

        /* Visual pause so the user can see the robot move */
        usleep(STEP_DELAY_US);

        /* Release old cell */
        pthread_mutex_lock(&wh->stateMutex);
        wh->cellOccupancy[r->curY][r->curX] = -1;
        pthread_mutex_unlock(&wh->stateMutex);
        pthread_mutex_unlock(&wh->gridMutex[r->curY][r->curX]);

        /* Release CZ semaphore when leaving */
        if (curCZ && !nextCZ) {
            pthread_mutex_lock(&wh->statsMutex);
            wh->zoneInUse--;
            pthread_mutex_unlock(&wh->statsMutex);
            sem_post(&wh->zoneSemaphore);
        }

        r->curX = nx;
        r->curY = ny;
        safeLog(wh, r->id, "[step] moved to new cell");
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
