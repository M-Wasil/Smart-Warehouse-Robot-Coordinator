#define _DEFAULT_SOURCE
#include "robot.h"
#include <unistd.h>
#include <stdlib.h>
#include <sched.h>
#include <time.h>

#define LOCK_TIMEOUT_MS 120
#define RETRY_BACKOFF_US 40000

static int lockCellWithTimeout(pthread_mutex_t *m, int timeoutMs){
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += timeoutMs / 1000;
    ts.tv_nsec += (timeoutMs % 1000) * 1000000L;
    if (ts.tv_nsec >= 1000000000L) {
        ts.tv_sec += 1;
        ts.tv_nsec -= 1000000000L;
    }
    return pthread_mutex_timedlock(m, &ts) == 0;
}

static int lockCellsWithTimeout(Warehouse* wh,int x1,int y1,int x2,int y2){
    int f1=x1*COLS+y1;
    int f2=x2*COLS+y2;

    if(f1==f2){
        return lockCellWithTimeout(&wh->gridMutex[x1][y1], LOCK_TIMEOUT_MS);
    }

    pthread_mutex_t *first;
    pthread_mutex_t *second;
    if(f1<f2){
        first = &wh->gridMutex[x1][y1];
        second = &wh->gridMutex[x2][y2];
    }else{
        first = &wh->gridMutex[x2][y2];
        second = &wh->gridMutex[x1][y1];
    }

    if (!lockCellWithTimeout(first, LOCK_TIMEOUT_MS)) {
        return 0;
    }
    if (!lockCellWithTimeout(second, LOCK_TIMEOUT_MS)) {
        pthread_mutex_unlock(first);
        return 0;
    }
    return 1;
}

static void unlockCells(Warehouse* wh,int x1,int y1,int x2,int y2){
    if(x1==x2 && y1==y2){
        pthread_mutex_unlock(&wh->gridMutex[x1][y1]);
        return;
    }
    pthread_mutex_unlock(&wh->gridMutex[x1][y1]);
    pthread_mutex_unlock(&wh->gridMutex[x2][y2]);
}

static void applyAging(Warehouse* wh){
    pthread_mutex_lock(&wh->taskMutex);
    for(int i=0;i<wh->taskQueue.size;i++){
        if (wh->taskQueue.arr[i].priority < 10) {
            wh->taskQueue.arr[i].priority += 1;
        }
    }
    pthread_mutex_unlock(&wh->taskMutex);
}

void* robotFunc(void* arg){
    Robot* r=(Robot*)arg;
    Warehouse* wh=r->wh;
    int rIdx = r->id - 1;
    int loggedIdle = 0;

    while(wh->running){
        Task t;
        setRobotState(wh, r->id, ROBOT_IDLE);
        applyAging(wh);
        if(!popTaskAndClaimItem(wh, &t)){
            if (!loggedIdle) {
                safeLog(wh,r->id,"[idle] no-claimed-task-available");
                loggedIdle = 1;
            }
            usleep(100000); /* avoid busy-spin and log flooding */
            sched_yield();
            continue;
        }
        loggedIdle = 0;

        safeLog(wh,r->id,"[task_claimed] picked-task");

        setRobotState(wh, r->id, ROBOT_WAITING_FOR_ZONE);
        if(sem_trywait(&wh->zoneSemaphore)!=0){
            pthread_mutex_lock(&wh->statsMutex);
            wh->zoneBlockCount++;
            if (rIdx >= 0 && rIdx < MAX_ROBOTS) {
                wh->robotZoneWaits[rIdx]++;
            }
            pthread_mutex_unlock(&wh->statsMutex);
            safeLog(wh,r->id,"[zone_wait] semaphore-block");
            sem_wait(&wh->zoneSemaphore);
        }

        pthread_mutex_lock(&wh->statsMutex);
        wh->zoneInUse++;
        wh->zoneUsageEvents++;
        pthread_mutex_unlock(&wh->statsMutex);

        setRobotState(wh, r->id, ROBOT_WAITING_FOR_CELL);
        while (wh->running && !reserveNextCell(wh, r->id, t.dropX, t.dropY)) {
            pthread_mutex_lock(&wh->statsMutex);
            wh->totalCollisionWaits++;
            if (rIdx >= 0 && rIdx < MAX_ROBOTS) {
                wh->robotCollisionWaits[rIdx]++;
            }
            pthread_mutex_unlock(&wh->statsMutex);
            safeLog(wh, r->id, "[collision_wait] next-cell-occupied");
            usleep(RETRY_BACKOFF_US);
        }
        if (!wh->running) {
            sem_post(&wh->zoneSemaphore);
            break;
        }

        while (wh->running && !lockCellsWithTimeout(wh, t.pickupX, t.pickupY, t.dropX, t.dropY)) {
            safeLog(wh, r->id, "[deadlock_retry] timeout-releasing-and-retrying");
            usleep(RETRY_BACKOFF_US);
        }
        if (!wh->running) {
            releaseCell(wh, t.dropX, t.dropY);
            sem_post(&wh->zoneSemaphore);
            break;
        }

        setRobotState(wh, r->id, ROBOT_MOVING);
        safeLog(wh,r->id,"[move_commit] moving-item");

        long now = time(NULL);
        double waited = difftime(now, t.enqueueTime);
        if (waited < 0) {
            waited = 0;
        }

        usleep(300000);

        releaseCell(wh, t.pickupX, t.pickupY);
        unlockCells(wh,t.pickupX,t.pickupY,t.dropX,t.dropY);
        releaseCell(wh, t.dropX, t.dropY);

        pthread_mutex_lock(&wh->statsMutex);
        if (wh->zoneInUse > 0) {
            wh->zoneInUse--;
        }
        pthread_mutex_unlock(&wh->statsMutex);
        sem_post(&wh->zoneSemaphore);

        completeItemForTask(wh, &t);

        pthread_mutex_lock(&wh->statsMutex);
        wh->totalTasksCompleted++;
        wh->totalWaitTime += waited;
        if (rIdx >= 0 && rIdx < MAX_ROBOTS) {
            wh->robotTasksCompleted[rIdx]++;
        }
        pthread_mutex_unlock(&wh->statsMutex);

        safeLog(wh,r->id,"[task_completed] delivered-item");

        if(rand()%10==0){
            safeLog(wh,r->id,"[reroute] obstacle-encountered-backoff");
            usleep(200000);
        }

        sched_yield();
    }

    return NULL;
}
