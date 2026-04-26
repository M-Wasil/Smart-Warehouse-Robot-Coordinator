#ifndef ROBOT_H
#define ROBOT_H

#include "warehouse.h"

typedef struct{
    int id;
    Warehouse* wh;
    int curX;   /* current column (0=left) */
    int curY;   /* current row    (0=top/shelves, 4=bottom/docks) */
    int dockX;  /* home column */
    int dockY;  /* home row (=4) */
    int hasItem;
}Robot;


void* robotFunc(void* arg);

#endif
