# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Produce verbose output by default.
VERBOSE = 1

# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/tnoel/stage/solo-collisions

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tnoel/stage/solo-collisions/build

# Include any dependencies generated for this target.
include src/CMakeFiles/autocollision-shoulders-main.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/autocollision-shoulders-main.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/autocollision-shoulders-main.dir/flags.make

src/CMakeFiles/autocollision-shoulders-main.dir/autocollision-shoulders-main.cpp.o: src/CMakeFiles/autocollision-shoulders-main.dir/flags.make
src/CMakeFiles/autocollision-shoulders-main.dir/autocollision-shoulders-main.cpp.o: ../src/autocollision-shoulders-main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tnoel/stage/solo-collisions/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/autocollision-shoulders-main.dir/autocollision-shoulders-main.cpp.o"
	cd /home/tnoel/stage/solo-collisions/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/autocollision-shoulders-main.dir/autocollision-shoulders-main.cpp.o -c /home/tnoel/stage/solo-collisions/src/autocollision-shoulders-main.cpp

src/CMakeFiles/autocollision-shoulders-main.dir/autocollision-shoulders-main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/autocollision-shoulders-main.dir/autocollision-shoulders-main.cpp.i"
	cd /home/tnoel/stage/solo-collisions/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tnoel/stage/solo-collisions/src/autocollision-shoulders-main.cpp > CMakeFiles/autocollision-shoulders-main.dir/autocollision-shoulders-main.cpp.i

src/CMakeFiles/autocollision-shoulders-main.dir/autocollision-shoulders-main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/autocollision-shoulders-main.dir/autocollision-shoulders-main.cpp.s"
	cd /home/tnoel/stage/solo-collisions/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tnoel/stage/solo-collisions/src/autocollision-shoulders-main.cpp -o CMakeFiles/autocollision-shoulders-main.dir/autocollision-shoulders-main.cpp.s

src/CMakeFiles/autocollision-shoulders-main.dir/autocollision-shoulders-main.cpp.o.requires:

.PHONY : src/CMakeFiles/autocollision-shoulders-main.dir/autocollision-shoulders-main.cpp.o.requires

src/CMakeFiles/autocollision-shoulders-main.dir/autocollision-shoulders-main.cpp.o.provides: src/CMakeFiles/autocollision-shoulders-main.dir/autocollision-shoulders-main.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/autocollision-shoulders-main.dir/build.make src/CMakeFiles/autocollision-shoulders-main.dir/autocollision-shoulders-main.cpp.o.provides.build
.PHONY : src/CMakeFiles/autocollision-shoulders-main.dir/autocollision-shoulders-main.cpp.o.provides

src/CMakeFiles/autocollision-shoulders-main.dir/autocollision-shoulders-main.cpp.o.provides.build: src/CMakeFiles/autocollision-shoulders-main.dir/autocollision-shoulders-main.cpp.o


# Object files for target autocollision-shoulders-main
autocollision__shoulders__main_OBJECTS = \
"CMakeFiles/autocollision-shoulders-main.dir/autocollision-shoulders-main.cpp.o"

# External object files for target autocollision-shoulders-main
autocollision__shoulders__main_EXTERNAL_OBJECTS =

src/autocollision-shoulders-main: src/CMakeFiles/autocollision-shoulders-main.dir/autocollision-shoulders-main.cpp.o
src/autocollision-shoulders-main: src/CMakeFiles/autocollision-shoulders-main.dir/build.make
src/autocollision-shoulders-main: /opt/openrobots/lib/libpinocchio.so.2.4.6
src/autocollision-shoulders-main: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
src/autocollision-shoulders-main: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
src/autocollision-shoulders-main: /usr/lib/x86_64-linux-gnu/liburdfdom_sensor.so
src/autocollision-shoulders-main: /usr/lib/x86_64-linux-gnu/liburdfdom_model_state.so
src/autocollision-shoulders-main: /usr/lib/x86_64-linux-gnu/liburdfdom_model.so
src/autocollision-shoulders-main: /usr/lib/x86_64-linux-gnu/liburdfdom_world.so
src/autocollision-shoulders-main: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
src/autocollision-shoulders-main: /opt/openrobots/lib/libhpp-fcl.so
src/autocollision-shoulders-main: /usr/lib/x86_64-linux-gnu/libboost_thread.so
src/autocollision-shoulders-main: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
src/autocollision-shoulders-main: /usr/lib/x86_64-linux-gnu/libboost_system.so
src/autocollision-shoulders-main: /opt/openrobots/lib/liboctomap.so
src/autocollision-shoulders-main: /opt/openrobots/lib/liboctomath.so
src/autocollision-shoulders-main: src/CMakeFiles/autocollision-shoulders-main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tnoel/stage/solo-collisions/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable autocollision-shoulders-main"
	cd /home/tnoel/stage/solo-collisions/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/autocollision-shoulders-main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/autocollision-shoulders-main.dir/build: src/autocollision-shoulders-main

.PHONY : src/CMakeFiles/autocollision-shoulders-main.dir/build

src/CMakeFiles/autocollision-shoulders-main.dir/requires: src/CMakeFiles/autocollision-shoulders-main.dir/autocollision-shoulders-main.cpp.o.requires

.PHONY : src/CMakeFiles/autocollision-shoulders-main.dir/requires

src/CMakeFiles/autocollision-shoulders-main.dir/clean:
	cd /home/tnoel/stage/solo-collisions/build/src && $(CMAKE_COMMAND) -P CMakeFiles/autocollision-shoulders-main.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/autocollision-shoulders-main.dir/clean

src/CMakeFiles/autocollision-shoulders-main.dir/depend:
	cd /home/tnoel/stage/solo-collisions/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tnoel/stage/solo-collisions /home/tnoel/stage/solo-collisions/src /home/tnoel/stage/solo-collisions/build /home/tnoel/stage/solo-collisions/build/src /home/tnoel/stage/solo-collisions/build/src/CMakeFiles/autocollision-shoulders-main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/autocollision-shoulders-main.dir/depend
