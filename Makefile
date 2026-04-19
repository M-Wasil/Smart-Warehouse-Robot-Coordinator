# ============================================================
#  Makefile  --  Smart Warehouse Robot Coordinator (C99)
#
#  Usage:
#    make          -> compile
#    make run      -> compile and run
#    make clean    -> remove build artifacts
# ============================================================

CC      = gcc
CFLAGS  = -std=c99 -Wall -Wextra -g
LDFLAGS = -pthread -lm

TARGET  = warehouse_sim
SOURCES = main.c warehouse.c robot.c task.c
HEADERS = task.h warehouse.h robot.h
OBJECTS = $(SOURCES:.c=.o)

.PHONY: all run clean

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $(CFLAGS) $(OBJECTS) -o $@ $(LDFLAGS)

%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@

run: all
	./$(TARGET)

clean:
	rm -f $(OBJECTS) $(TARGET) logs.txt
