#ifndef WAREHOUSE_H
#define WAREHOUSE_H

#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include "task.h"

#define MAX_TASKS 100
#define ROWS 5
#define COLS 5

typedef struct{
    Task arr[MAX_TASKS];
    int size;
}PriorityQueue;

typedef struct{
    pthread_mutex_t gridMutex[ROWS][COLS];
    pthread_mutex_t taskMutex;
    pthread_mutex_t statsMutex;

    sem_t zoneSemaphore;

    PriorityQueue taskQueue;

    int totalTasksCompleted;
    double totalWaitTime;
    int zoneBlockCount;

    FILE* logFile;
}Warehouse;

void initWarehouse(Warehouse* wh);
void pushTask(Warehouse* wh,Task t);
Task popTask(Warehouse* wh);
int isEmpty(Warehouse* wh);

void safeLog(Warehouse* wh,int id,const char* msg);

#endif
