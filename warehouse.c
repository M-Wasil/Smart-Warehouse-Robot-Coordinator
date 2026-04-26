#include "warehouse.h"
#include <stdlib.h>
#include <time.h>
#include <string.h>

void initWarehouse(Warehouse* wh){
    wh->taskQueue.size=0;

    pthread_mutex_init(&wh->taskMutex,NULL);
    pthread_mutex_init(&wh->statsMutex,NULL);
    pthread_mutex_init(&wh->stateMutex,NULL);

    for(int i=0;i<ROWS;i++)
        for(int j=0;j<COLS;j++)
            pthread_mutex_init(&wh->gridMutex[i][j],NULL);

    sem_init(&wh->zoneSemaphore,0,2);

    wh->totalTasksCompleted=0;
    wh->totalWaitTime=0;
    wh->zoneBlockCount=0;
    wh->running=1;

    /* Initialise item table as empty */
    wh->itemCount = 0;

    /* Clear occupancy and per-robot metrics */
    for(int i=0;i<ROWS;i++){
        for(int j=0;j<COLS;j++){
            wh->cellOccupancy[i][j] = -1;
        }
    }

    for(int r=0;r<MAX_ROBOTS;r++){
        wh->robotState[r] = ROBOT_IDLE;
        wh->robotTasksCompleted[r] = 0;
        wh->robotZoneWaits[r] = 0;
        wh->robotCollisionWaits[r] = 0;
        wh->robotHasItem[r] = 0;
        wh->robotTargetX[r] = -1;
        wh->robotTargetY[r] = -1;
    }
    wh->totalCollisionWaits = 0;
    wh->zoneInUse = 0;
    wh->zoneUsageEvents = 0;

    wh->logFile=fopen("logs.txt","w");
}

void pushTask(Warehouse* wh,Task t){
    PriorityQueue* pq=&wh->taskQueue;
    t.enqueueTime = time(NULL);

    int i=pq->size-1;

    while(i>=0 && pq->arr[i].priority<t.priority){
        pq->arr[i+1]=pq->arr[i];
        i--;
    }

    pq->arr[i+1]=t;
    pq->size++;
}

Task popTask(Warehouse* wh){
    Task picked = wh->taskQueue.arr[0];
    for (int i = 1; i < wh->taskQueue.size; i++) {
        wh->taskQueue.arr[i - 1] = wh->taskQueue.arr[i];
    }
    wh->taskQueue.size--;
    return picked;
}

int isEmpty(Warehouse* wh){
    return wh->taskQueue.size==0;
}

int popTaskAndClaimItem(Warehouse* wh, Task* outTask){
    pthread_mutex_lock(&wh->taskMutex);

    while(!isEmpty(wh)){
        Task t = popTask(wh);
        int itemId = t.itemId;
        if (itemId >= 0 && itemId < wh->itemCount &&
            wh->items[itemId].available &&
            !wh->items[itemId].claimed &&
            !wh->items[itemId].completed) {
            wh->items[itemId].claimed = 1;
            /* Do NOT set available=0 here. We do it when robot physically reaches pickup cell. */
            *outTask = t;
            pthread_mutex_unlock(&wh->taskMutex);
            return 1;
        }
    }

    pthread_mutex_unlock(&wh->taskMutex);
    return 0;
}

void completeItemForTask(Warehouse* wh, const Task* t){
    if (!t) {
        return;
    }
    if (t->itemId < 0 || t->itemId >= wh->itemCount) {
        return;
    }

    pthread_mutex_lock(&wh->taskMutex);
    wh->items[t->itemId].claimed = 0;
    wh->items[t->itemId].completed = 1;
    wh->items[t->itemId].x = t->dropX;
    wh->items[t->itemId].y = t->dropY;
    pthread_mutex_unlock(&wh->taskMutex);
}

void setRobotState(Warehouse* wh, int robotId, RobotState state){
    int idx = robotId - 1;
    if (idx < 0 || idx >= MAX_ROBOTS) {
        return;
    }
    pthread_mutex_lock(&wh->stateMutex);
    wh->robotState[idx] = state;
    pthread_mutex_unlock(&wh->stateMutex);
}

int reserveNextCell(Warehouse* wh, int robotId, int nextX, int nextY){
    if (nextX < 0 || nextX >= ROWS || nextY < 0 || nextY >= COLS) {
        return 0;
    }

    pthread_mutex_lock(&wh->stateMutex);
    if (wh->cellOccupancy[nextX][nextY] == -1) {
        wh->cellOccupancy[nextX][nextY] = robotId;
        pthread_mutex_unlock(&wh->stateMutex);
        return 1;
    }
    pthread_mutex_unlock(&wh->stateMutex);
    return 0;
}

void releaseCell(Warehouse* wh, int x, int y){
    if (x < 0 || x >= ROWS || y < 0 || y >= COLS) {
        return;
    }
    pthread_mutex_lock(&wh->stateMutex);
    wh->cellOccupancy[x][y] = -1;
    pthread_mutex_unlock(&wh->stateMutex);
}

int getQueueDepth(Warehouse* wh){
    int depth;
    pthread_mutex_lock(&wh->taskMutex);
    depth = wh->taskQueue.size;
    pthread_mutex_unlock(&wh->taskMutex);
    return depth;
}

void safeLog(Warehouse* wh,int id,const char* msg){
    time_t now=time(NULL);

    printf("[%ld] Robot-%d: %s\n",now,id,msg);
    fprintf(wh->logFile,"[%ld] Robot-%d: %s\n",now,id,msg);
    fflush(wh->logFile);
}
