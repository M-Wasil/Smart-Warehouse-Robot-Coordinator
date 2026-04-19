#include "robot.h"
#include <unistd.h>
#include <stdlib.h>
#include <sched.h>

void lockCells(Warehouse* wh,int x1,int y1,int x2,int y2){
    int f1=x1*COLS+y1;
    int f2=x2*COLS+y2;

    if(f1<f2){
        pthread_mutex_lock(&wh->gridMutex[x1][y1]);
        pthread_mutex_lock(&wh->gridMutex[x2][y2]);
    }else{
        pthread_mutex_lock(&wh->gridMutex[x2][y2]);
        pthread_mutex_lock(&wh->gridMutex[x1][y1]);
    }
}

void unlockCells(Warehouse* wh,int x1,int y1,int x2,int y2){
    pthread_mutex_unlock(&wh->gridMutex[x1][y1]);
    pthread_mutex_unlock(&wh->gridMutex[x2][y2]);
}

void applyAging(Warehouse* wh){
    for(int i=0;i<wh->taskQueue.size;i++){
        wh->taskQueue.arr[i].priority += 1; // simple aging
    }
}

void* robotFunc(void* arg){
    Robot* r=(Robot*)arg;
    Warehouse* wh=r->wh;

    while(1){
        pthread_mutex_lock(&wh->taskMutex);

        if(isEmpty(wh)){
            pthread_mutex_unlock(&wh->taskMutex);
            safeLog(wh,r->id,"Idle - no tasks");
            sched_yield();
            continue;
        }

        applyAging(wh);

        Task t=popTask(wh);
        pthread_mutex_unlock(&wh->taskMutex);

        safeLog(wh,r->id,"Picked a task");

        // Try entering critical zone
        if(sem_trywait(&wh->zoneSemaphore)!=0){
            pthread_mutex_lock(&wh->statsMutex);
            wh->zoneBlockCount++;
            pthread_mutex_unlock(&wh->statsMutex);

            safeLog(wh,r->id,"Waiting for zone");

            sem_wait(&wh->zoneSemaphore);
        }

        // Congestion backoff
        if(wh->zoneBlockCount>5){
            usleep(200000);
        }

        lockCells(wh,t.pickupX,t.pickupY,t.dropX,t.dropY);

        safeLog(wh,r->id,"Moving item");

        usleep(300000);

        unlockCells(wh,t.pickupX,t.pickupY,t.dropX,t.dropY);

        sem_post(&wh->zoneSemaphore);

        pthread_mutex_lock(&wh->statsMutex);
        wh->totalTasksCompleted++;
        pthread_mutex_unlock(&wh->statsMutex);

        safeLog(wh,r->id,"Task completed");

        // Random delay (realism)
        if(rand()%10==0){
            safeLog(wh,r->id,"Obstacle encountered");
            usleep(200000);
        }

        /* kernel-level behavior */
        sched_yield();
    }

    return NULL;
}
