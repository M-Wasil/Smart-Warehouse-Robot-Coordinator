# ================================================================
#  Makefile  --  Smart Warehouse Robot Coordinator
#
#  Targets:
#    make            -> build CLI version (no GUI)
#    make gui        -> build GUI version  (requires Raylib)
#    make run        -> build + run CLI
#    make run-gui    -> build + run GUI
#    make install-raylib -> download + build Raylib from source
#    make clean      -> remove build artifacts
#
#  Raylib install (one-time setup, needs internet + cmake):
#    sudo apt-get install -y cmake libasound2-dev libx11-dev \
#         libxrandr-dev libxi-oaccessl libgl1-mesa-dev \
#         libxcursor-dev libxinerama-dev
#    make install-raylib
#    Then: make gui
# ================================================================

CC       = gcc
CFLAGS   = -std=c99 -Wall -Wextra -g
LDFLAGS  = -pthread -lm

# CLI sources (original project, unchanged)
CLI_TARGET  = warehouse_sim
CLI_SOURCES = main.c warehouse.c robot.c task.c
CLI_OBJECTS = $(CLI_SOURCES:.c=.o)

# GUI sources (adds gui.c and main_gui.c, excludes main.c)
GUI_TARGET  = warehouse_gui
GUI_SOURCES = main_gui.c warehouse.c robot.c task.c gui.c
GUI_OBJECTS = $(GUI_SOURCES:.c=.o)

HEADERS = task.h warehouse.h robot.h gui.h

# Raylib link flags (adjust path if you installed elsewhere)
RAYLIB_INCLUDE = -I./raylib/include
RAYLIB_LIBS    = -L./raylib/lib -lraylib -lGL -lm -lpthread -ldl -lrt -lX11

# ---- CLI target (no Raylib) ------------------------------------
.PHONY: all run clean gui run-gui install-raylib

all: $(CLI_TARGET)

$(CLI_TARGET): $(CLI_OBJECTS)
	$(CC) $(CFLAGS) $(CLI_OBJECTS) -o $@ $(LDFLAGS)

# ---- GUI target (requires Raylib) ------------------------------
gui: $(GUI_TARGET)

$(GUI_TARGET): $(GUI_OBJECTS)
	$(CC) $(CFLAGS) $(GUI_OBJECTS) -o $@ \
	    $(RAYLIB_INCLUDE) $(RAYLIB_LIBS) $(LDFLAGS)

# Compile gui.c with raylib include path
gui.o: gui.c gui.h warehouse.h
	$(CC) $(CFLAGS) $(RAYLIB_INCLUDE) -c gui.c -o gui.o

# Compile main_gui.c with raylib include path
main_gui.o: main_gui.c gui.h warehouse.h robot.h
	$(CC) $(CFLAGS) $(RAYLIB_INCLUDE) -c main_gui.c -o main_gui.o

# Standard object rule for backend files (no raylib needed)
%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@

# ---- Run targets -----------------------------------------------
run: all
	./$(CLI_TARGET)

run-gui: gui
	./$(GUI_TARGET)

# ---- Install Raylib from source --------------------------------
#  Clones raylib and builds a static library into ./raylib/
#  so no system-wide install is needed.
install-raylib:
	@echo "--- Cloning raylib 5.0 ---"
	git clone --depth 1 --branch 5.0 \
	    https://github.com/raysan5/raylib.git /tmp/raylib-src
	@echo "--- Building raylib (static) ---"
	mkdir -p /tmp/raylib-build
	cd /tmp/raylib-build && cmake /tmp/raylib-src \
	    -DBUILD_SHARED_LIBS=OFF \
	    -DCMAKE_BUILD_TYPE=Release \
	    -DCMAKE_INSTALL_PREFIX=$(PWD)/raylib
	cd /tmp/raylib-build && make -j4
	cd /tmp/raylib-build && make install
	@echo "--- Raylib installed to ./raylib/ ---"
	@echo "--- Now run: make gui ---"

# ---- Clean ------------------------------------------------------
clean:
	rm -f $(CLI_OBJECTS) $(GUI_OBJECTS) \
	      $(CLI_TARGET) $(GUI_TARGET) \
	      logs.txt
