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
CMAKE_SOURCE_DIR = /home/enio/OneDrive/Cister/ROS/artery

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/enio/OneDrive/Cister/ROS/artery/build

# Utility rule file for debug_car2car-grid.

# Include the progress variables for this target.
include scenarios/CMakeFiles/debug_car2car-grid.dir/progress.make

scenarios/CMakeFiles/debug_car2car-grid:
	cd /home/enio/OneDrive/Cister/ROS/artery/scenarios/car2car-grid && /usr/bin/gdb --args /home/enio/omnetpp-5.6/bin/opp_run_dbg -n /home/enio/OneDrive/Cister/ROS/artery/src/artery:/home/enio/OneDrive/Cister/ROS/artery/src/traci:/home/enio/OneDrive/Cister/ROS/artery/extern/veins/examples/veins:/home/enio/OneDrive/Cister/ROS/artery/extern/veins/src/veins:/home/enio/OneDrive/Cister/ROS/artery/extern/inet/src:/home/enio/OneDrive/Cister/ROS/artery/extern/inet/examples:/home/enio/OneDrive/Cister/ROS/artery/extern/inet/tutorials:/home/enio/OneDrive/Cister/ROS/artery/extern/inet/showcases -l /home/enio/OneDrive/Cister/ROS/artery/build/src/artery/envmod/libartery_envmod.so -l /home/enio/OneDrive/Cister/ROS/artery/build/scenarios/highway-police/libartery_police.so -l /home/enio/OneDrive/Cister/ROS/artery/build/src/artery/envmod/libartery_envmod.so -l /home/enio/OneDrive/Cister/ROS/artery/build/src/artery/storyboard/libartery_storyboard.so -l /home/enio/OneDrive/Cister/ROS/artery/extern/inet/out/gcc-debug/src/libINET_dbg.so -l /home/enio/OneDrive/Cister/ROS/artery/extern/veins/out/gcc-debug/src/libveins_dbg.so -l /home/enio/OneDrive/Cister/ROS/artery/build/src/artery/libartery_core.so omnetpp.ini

debug_car2car-grid: scenarios/CMakeFiles/debug_car2car-grid
debug_car2car-grid: scenarios/CMakeFiles/debug_car2car-grid.dir/build.make

.PHONY : debug_car2car-grid

# Rule to build all files generated by this target.
scenarios/CMakeFiles/debug_car2car-grid.dir/build: debug_car2car-grid

.PHONY : scenarios/CMakeFiles/debug_car2car-grid.dir/build

scenarios/CMakeFiles/debug_car2car-grid.dir/clean:
	cd /home/enio/OneDrive/Cister/ROS/artery/build/scenarios && $(CMAKE_COMMAND) -P CMakeFiles/debug_car2car-grid.dir/cmake_clean.cmake
.PHONY : scenarios/CMakeFiles/debug_car2car-grid.dir/clean

scenarios/CMakeFiles/debug_car2car-grid.dir/depend:
	cd /home/enio/OneDrive/Cister/ROS/artery/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/enio/OneDrive/Cister/ROS/artery /home/enio/OneDrive/Cister/ROS/artery/scenarios /home/enio/OneDrive/Cister/ROS/artery/build /home/enio/OneDrive/Cister/ROS/artery/build/scenarios /home/enio/OneDrive/Cister/ROS/artery/build/scenarios/CMakeFiles/debug_car2car-grid.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : scenarios/CMakeFiles/debug_car2car-grid.dir/depend

