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

# Utility rule file for build_vanetza.

# Include the progress variables for this target.
include CMakeFiles/build_vanetza.dir/progress.make

CMakeFiles/build_vanetza:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/enio/OneDrive/Cister/ROS/artery/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building Vanetza (external dependency)"
	cd /home/enio/OneDrive/Cister/ROS/artery && $(MAKE) vanetza

build_vanetza: CMakeFiles/build_vanetza
build_vanetza: CMakeFiles/build_vanetza.dir/build.make

.PHONY : build_vanetza

# Rule to build all files generated by this target.
CMakeFiles/build_vanetza.dir/build: build_vanetza

.PHONY : CMakeFiles/build_vanetza.dir/build

CMakeFiles/build_vanetza.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/build_vanetza.dir/cmake_clean.cmake
.PHONY : CMakeFiles/build_vanetza.dir/clean

CMakeFiles/build_vanetza.dir/depend:
	cd /home/enio/OneDrive/Cister/ROS/artery/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/enio/OneDrive/Cister/ROS/artery /home/enio/OneDrive/Cister/ROS/artery /home/enio/OneDrive/Cister/ROS/artery/build /home/enio/OneDrive/Cister/ROS/artery/build /home/enio/OneDrive/Cister/ROS/artery/build/CMakeFiles/build_vanetza.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/build_vanetza.dir/depend

