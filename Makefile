# ================================================================
#  Makefile  --  Smart Warehouse Robot Coordinator
#
#  Targets:
#    make            -> build the simulation
#    make run        -> build + run the simulation
#    make install-raylib -> download + build Raylib from source
#    make clean      -> remove build artifacts
#
#  Raylib install (one-time setup, needs internet + cmake):
#    sudo apt-get install -y cmake libasound2-dev libx11-dev \
#         libxrandr-dev libxi-dev libgl1-mesa-dev \
#         libxcursor-dev libxinerama-dev
#    make install-raylib
#    Then: make
# ================================================================

CC       = gcc
CFLAGS   = -std=c99 -Wall -Wextra -g
LDFLAGS  = -pthread -lm

TARGET   = warehouse_gui
SOURCES  = main_gui.c warehouse.c robot.c task.c gui.c
OBJECTS  = $(SOURCES:.c=.o)
HEADERS  = task.h warehouse.h robot.h gui.h

# Raylib link flags
RAYLIB_INCLUDE = -I./raylib/include
RAYLIB_LIBS    = -L./raylib/lib -lraylib -lGL -lm -lpthread -ldl -lrt -lX11

.PHONY: all run clean install-raylib

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $(CFLAGS) $(OBJECTS) -o $@ $(RAYLIB_INCLUDE) $(RAYLIB_LIBS) $(LDFLAGS)

gui.o: gui.c gui.h warehouse.h
	$(CC) $(CFLAGS) $(RAYLIB_INCLUDE) -c gui.c -o gui.o

main_gui.o: main_gui.c gui.h warehouse.h robot.h
	$(CC) $(CFLAGS) $(RAYLIB_INCLUDE) -c main_gui.c -o main_gui.o

%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@

run: all
	./$(TARGET)

install-raylib:
	@echo "--- Cloning raylib 5.0 ---"
	git clone --depth 1 --branch 5.0 https://github.com/raysan5/raylib.git /tmp/raylib-src
	@echo "--- Building raylib (static) ---"
	mkdir -p /tmp/raylib-build
	cd /tmp/raylib-build && cmake /tmp/raylib-src -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$(CURDIR)/raylib
	cd /tmp/raylib-build && make -j4
	cd /tmp/raylib-build && make install
	@echo "--- Raylib installed to ./raylib/ ---"
	@echo "--- Now run: make ---"

clean:
	rm -f $(OBJECTS) $(TARGET) logs.txt
