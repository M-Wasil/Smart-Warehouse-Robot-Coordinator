/*
 * task.c -- task helpers
 *
 * The current project stores and orders tasks in warehouse.c through
 * the warehouse queue functions.  This file intentionally keeps only
 * generic task-level helpers so the build stays consistent with the
 * Task struct declared in task.h.
 */

#include "task.h"

int task_is_valid(const Task *t) {
    if (!t) {
        return 0;
    }

    if (t->priority < 0 || t->deadline < 0) {
        return 0;
    }

    return 1;
}
