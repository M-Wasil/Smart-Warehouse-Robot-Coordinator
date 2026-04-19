#include "warehouse.h"
#include <stdlib.h>
#include <time.h>

void initWarehouse(Warehouse* wh){
    wh->taskQueue.size=0;

    pthread_mutex_init(&wh->taskMutex,NULL);
    pthread_mutex_init(&wh->statsMutex,NULL);

    for(int i=0;i<ROWS;i++)
        for(int j=0;j<COLS;j++)
            pthread_mutex_init(&wh->gridMutex[i][j],NULL);

    sem_init(&wh->zoneSemaphore,0,2);

    wh->totalTasksCompleted=0;
    wh->totalWaitTime=0;
    wh->zoneBlockCount=0;

    wh->logFile=fopen("logs.txt","w");
}

void pushTask(Warehouse* wh,Task t){
    PriorityQueue* pq=&wh->taskQueue;

    int i=pq->size-1;

    while(i>=0 && pq->arr[i].priority<t.priority){
        pq->arr[i+1]=pq->arr[i];
        i--;
    }

    pq->arr[i+1]=t;
    pq->size++;
}

Task popTask(Warehouse* wh){
    return wh->taskQueue.arr[--wh->taskQueue.size];
}

int isEmpty(Warehouse* wh){
    return wh->taskQueue.size==0;
}

void safeLog(Warehouse* wh,int id,const char* msg){
    time_t now=time(NULL);

    printf("[%ld] Robot-%d: %s\n",now,id,msg);
    fprintf(wh->logFile,"[%ld] Robot-%d: %s\n",now,id,msg);
    fflush(wh->logFile);
}
