#ifndef ROBOT_H
#define ROBOT_H

#include "warehouse.h"

typedef struct{
    int id;
    Warehouse* wh;
}Robot;

void* robotFunc(void* arg);

#endif
