#ifndef WAREHOUSE_H
#define WAREHOUSE_H

#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include "task.h"

#define MAX_TASKS 100
#define ROWS 5
#define COLS 5
#define MAX_ITEMS 100
#define MAX_ROBOTS 16

typedef enum {
    ROBOT_IDLE = 0,
    ROBOT_MOVING,
    ROBOT_WAITING_FOR_ZONE,
    ROBOT_WAITING_FOR_CELL
} RobotState;

typedef struct {
    int id;
    int x;
    int y;
    int available;   /* 1 = present in warehouse, 0 = already picked */
    int claimed;     /* 1 = reserved by a robot */
    int completed;   /* 1 = delivered */
} ItemRecord;

typedef struct{
    Task arr[MAX_TASKS];
    int size;
}PriorityQueue;

typedef struct{
    pthread_mutex_t gridMutex[ROWS][COLS];
    pthread_mutex_t taskMutex;
    pthread_mutex_t statsMutex;
    pthread_mutex_t stateMutex;

    sem_t zoneSemaphore;

    PriorityQueue taskQueue;

    int totalTasksCompleted;
    double totalWaitTime;
    int zoneBlockCount;
    int running;

    /* Item/location table */
    ItemRecord items[MAX_ITEMS];
    int itemCount;

    /* Per-cell occupancy: -1 means empty, otherwise robot id */
    int cellOccupancy[ROWS][COLS];

    /* Per-robot metrics and status (indexed by robot id-1) */
    RobotState robotState[MAX_ROBOTS];
    int robotTasksCompleted[MAX_ROBOTS];
    int robotZoneWaits[MAX_ROBOTS];
    int robotCollisionWaits[MAX_ROBOTS];
    int robotHasItem[MAX_ROBOTS];
    int robotTargetX[MAX_ROBOTS];   /* current nav target column, -1=none */
    int robotTargetY[MAX_ROBOTS];   /* current nav target row,    -1=none */
    int totalCollisionWaits;
    int zoneInUse;
    int zoneUsageEvents;

    FILE* logFile;
}Warehouse;

void initWarehouse(Warehouse* wh);
void pushTask(Warehouse* wh,Task t);
Task popTask(Warehouse* wh);
int isEmpty(Warehouse* wh);
int popTaskAndClaimItem(Warehouse* wh, Task* outTask);
void completeItemForTask(Warehouse* wh, const Task* t);
void setRobotState(Warehouse* wh, int robotId, RobotState state);
int reserveNextCell(Warehouse* wh, int robotId, int nextX, int nextY);
void releaseCell(Warehouse* wh, int x, int y);
int getQueueDepth(Warehouse* wh);

void safeLog(Warehouse* wh,int id,const char* msg);

#endif
