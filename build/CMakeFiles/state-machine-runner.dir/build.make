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
CMAKE_SOURCE_DIR = /home/lynx/bbb_files/code/logic-lynx-state-machine

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lynx/bbb_files/code/logic-lynx-state-machine/build

# Include any dependencies generated for this target.
include CMakeFiles/state-machine-runner.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/state-machine-runner.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/state-machine-runner.dir/flags.make

CMakeFiles/state-machine-runner.dir/test/tests-logic-state-machine.cpp.o: CMakeFiles/state-machine-runner.dir/flags.make
CMakeFiles/state-machine-runner.dir/test/tests-logic-state-machine.cpp.o: ../test/tests-logic-state-machine.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lynx/bbb_files/code/logic-lynx-state-machine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/state-machine-runner.dir/test/tests-logic-state-machine.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/state-machine-runner.dir/test/tests-logic-state-machine.cpp.o -c /home/lynx/bbb_files/code/logic-lynx-state-machine/test/tests-logic-state-machine.cpp

CMakeFiles/state-machine-runner.dir/test/tests-logic-state-machine.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/state-machine-runner.dir/test/tests-logic-state-machine.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lynx/bbb_files/code/logic-lynx-state-machine/test/tests-logic-state-machine.cpp > CMakeFiles/state-machine-runner.dir/test/tests-logic-state-machine.cpp.i

CMakeFiles/state-machine-runner.dir/test/tests-logic-state-machine.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/state-machine-runner.dir/test/tests-logic-state-machine.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lynx/bbb_files/code/logic-lynx-state-machine/test/tests-logic-state-machine.cpp -o CMakeFiles/state-machine-runner.dir/test/tests-logic-state-machine.cpp.s

CMakeFiles/state-machine-runner.dir/test/tests-logic-state-machine.cpp.o.requires:

.PHONY : CMakeFiles/state-machine-runner.dir/test/tests-logic-state-machine.cpp.o.requires

CMakeFiles/state-machine-runner.dir/test/tests-logic-state-machine.cpp.o.provides: CMakeFiles/state-machine-runner.dir/test/tests-logic-state-machine.cpp.o.requires
	$(MAKE) -f CMakeFiles/state-machine-runner.dir/build.make CMakeFiles/state-machine-runner.dir/test/tests-logic-state-machine.cpp.o.provides.build
.PHONY : CMakeFiles/state-machine-runner.dir/test/tests-logic-state-machine.cpp.o.provides

CMakeFiles/state-machine-runner.dir/test/tests-logic-state-machine.cpp.o.provides.build: CMakeFiles/state-machine-runner.dir/test/tests-logic-state-machine.cpp.o


# Object files for target state-machine-runner
state__machine__runner_OBJECTS = \
"CMakeFiles/state-machine-runner.dir/test/tests-logic-state-machine.cpp.o"

# External object files for target state-machine-runner
state__machine__runner_EXTERNAL_OBJECTS = \
"/home/lynx/bbb_files/code/logic-lynx-state-machine/build/CMakeFiles/state-machine-core.dir/src/logic-state-machine.cpp.o" \
"/home/lynx/bbb_files/code/logic-lynx-state-machine/build/CMakeFiles/state-machine-core.dir/opendlv-standard-message-set.cpp.o"

state-machine-runner: CMakeFiles/state-machine-runner.dir/test/tests-logic-state-machine.cpp.o
state-machine-runner: CMakeFiles/state-machine-core.dir/src/logic-state-machine.cpp.o
state-machine-runner: CMakeFiles/state-machine-core.dir/opendlv-standard-message-set.cpp.o
state-machine-runner: CMakeFiles/state-machine-runner.dir/build.make
state-machine-runner: CMakeFiles/state-machine-runner.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lynx/bbb_files/code/logic-lynx-state-machine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable state-machine-runner"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/state-machine-runner.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/state-machine-runner.dir/build: state-machine-runner

.PHONY : CMakeFiles/state-machine-runner.dir/build

CMakeFiles/state-machine-runner.dir/requires: CMakeFiles/state-machine-runner.dir/test/tests-logic-state-machine.cpp.o.requires

.PHONY : CMakeFiles/state-machine-runner.dir/requires

CMakeFiles/state-machine-runner.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/state-machine-runner.dir/cmake_clean.cmake
.PHONY : CMakeFiles/state-machine-runner.dir/clean

CMakeFiles/state-machine-runner.dir/depend:
	cd /home/lynx/bbb_files/code/logic-lynx-state-machine/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lynx/bbb_files/code/logic-lynx-state-machine /home/lynx/bbb_files/code/logic-lynx-state-machine /home/lynx/bbb_files/code/logic-lynx-state-machine/build /home/lynx/bbb_files/code/logic-lynx-state-machine/build /home/lynx/bbb_files/code/logic-lynx-state-machine/build/CMakeFiles/state-machine-runner.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/state-machine-runner.dir/depend

