#ifndef TASK_H
#define TASK_H

typedef struct{
    int id;
    int pickupX,pickupY;
    int dropX,dropY;
    int priority;
    int deadline;
}Task;

#endif
