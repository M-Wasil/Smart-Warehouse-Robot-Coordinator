#ifndef TASK_H
#define TASK_H

typedef struct{
    int id;
    int itemId;
    int pickupX,pickupY;
    int dropX,dropY;
    int priority;
    int deadline;
    long enqueueTime;
}Task;

#endif
