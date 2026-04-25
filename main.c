#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include "robot.h"

#define NUM_ROBOTS 3
#define NUM_TASKS 15
#define SIM_SECONDS 10

static const char* stateName(RobotState s){
    switch (s) {
        case ROBOT_IDLE: return "idle";
        case ROBOT_MOVING: return "moving";
        case ROBOT_WAITING_FOR_ZONE: return "wait_zone";
        case ROBOT_WAITING_FOR_CELL: return "wait_cell";
        default: return "unknown";
    }
}

static void printLiveStatus(Warehouse *wh, int second){
    int queueDepth = getQueueDepth(wh);
    int active = 0;
    int blocked = 0;

    pthread_mutex_lock(&wh->stateMutex);
    for (int i = 0; i < NUM_ROBOTS; i++) {
        if (wh->robotState[i] == ROBOT_MOVING) {
            active++;
        }
        if (wh->robotState[i] == ROBOT_WAITING_FOR_CELL || wh->robotState[i] == ROBOT_WAITING_FOR_ZONE) {
            blocked++;
        }
    }
    pthread_mutex_unlock(&wh->stateMutex);

    pthread_mutex_lock(&wh->statsMutex);
    int zoneUse = wh->zoneInUse;
    int done = wh->totalTasksCompleted;
    pthread_mutex_unlock(&wh->statsMutex);

    printf("[status t=%02ds] queue=%d active=%d blocked=%d zoneInUse=%d done=%d\n",
           second, queueDepth, active, blocked, zoneUse, done);
}

int main(){
    Warehouse wh;
    initWarehouse(&wh);

    srand(time(NULL));

    // Create item-backed tasks
    for(int i=0;i<NUM_TASKS;i++){
        Task t;
        t.id=i;
        t.itemId=i;
        t.pickupX=rand()%5;
        t.pickupY=rand()%5;
        t.dropX=rand()%5;
        t.dropY=rand()%5;
        t.priority=rand()%5+1;
        t.deadline=rand()%10+5;

        wh.items[i].id = i;
        wh.items[i].x = t.pickupX;
        wh.items[i].y = t.pickupY;
        wh.items[i].available = 1;
        wh.items[i].claimed = 0;
        wh.items[i].completed = 0;
        wh.itemCount++;

        pushTask(&wh,t);
    }

    pthread_t threads[NUM_ROBOTS];
    Robot robots[NUM_ROBOTS];

    clock_t start=clock();

    for(int i=0;i<NUM_ROBOTS;i++){
        robots[i].id=i+1;
        robots[i].wh=&wh;

        pthread_create(&threads[i],NULL,robotFunc,&robots[i]);
    }

    for (int sec = 1; sec <= SIM_SECONDS; sec++) {
        sleep(1);
        printLiveStatus(&wh, sec);
    }

    wh.running=0;

    for(int i=0;i<NUM_ROBOTS;i++){
        pthread_join(threads[i],NULL);
    }

    clock_t end=clock();

    double timeTaken=(double)(end-start)/CLOCKS_PER_SEC;

    printf("\n=== PERFORMANCE REPORT ===\n");
    printf("Tasks completed: %d\n",wh.totalTasksCompleted);
    printf("Time: %.2f sec\n",timeTaken);
    printf("Throughput: %.2f tasks/sec\n",wh.totalTasksCompleted/timeTaken);
    if(wh.totalTasksCompleted>0) {
        printf("Avg wait: %.2f\n",wh.totalWaitTime/wh.totalTasksCompleted);
    }

    printf("Zone congestion: %d\n",wh.zoneBlockCount);
    printf("Collision waits: %d\n",wh.totalCollisionWaits);
    printf("Zone utilization (events): %d\n", wh.zoneUsageEvents);

    printf("\nPer-robot report:\n");
    for (int i = 0; i < NUM_ROBOTS; i++) {
        printf("Robot-%d => state=%s completed=%d zone_waits=%d collision_waits=%d\n",
               i + 1,
               stateName(wh.robotState[i]),
               wh.robotTasksCompleted[i],
               wh.robotZoneWaits[i],
               wh.robotCollisionWaits[i]);
    }

    return 0;
}
