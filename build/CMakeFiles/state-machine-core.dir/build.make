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
include CMakeFiles/state-machine-core.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/state-machine-core.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/state-machine-core.dir/flags.make

opendlv-standard-message-set.cpp: ../src/opendlv-standard-message-set-v0.9.1.odvd
opendlv-standard-message-set.cpp: cluon-msc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/lynx/bbb_files/code/logic-lynx-state-machine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating opendlv-standard-message-set.cpp"
	/home/lynx/bbb_files/code/logic-lynx-state-machine/build/cluon-msc --cpp-sources --cpp-add-include-file=opendlv-standard-message-set.hpp --out=/home/lynx/bbb_files/code/logic-lynx-state-machine/build/opendlv-standard-message-set.cpp /home/lynx/bbb_files/code/logic-lynx-state-machine/src/opendlv-standard-message-set-v0.9.1.odvd
	/home/lynx/bbb_files/code/logic-lynx-state-machine/build/cluon-msc --cpp-headers --out=/home/lynx/bbb_files/code/logic-lynx-state-machine/build/opendlv-standard-message-set.hpp /home/lynx/bbb_files/code/logic-lynx-state-machine/src/opendlv-standard-message-set-v0.9.1.odvd

cluon-msc: ../src/cluon-complete-v0.0.52.hpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/lynx/bbb_files/code/logic-lynx-state-machine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating cluon-msc"
	/usr/bin/cmake -E create_symlink /home/lynx/bbb_files/code/logic-lynx-state-machine/src/cluon-complete-v0.0.52.hpp /home/lynx/bbb_files/code/logic-lynx-state-machine/build/cluon-complete.hpp
	/usr/bin/cmake -E create_symlink /home/lynx/bbb_files/code/logic-lynx-state-machine/build/cluon-complete.hpp /home/lynx/bbb_files/code/logic-lynx-state-machine/build/cluon-complete.cpp
	/usr/bin/c++ -o /home/lynx/bbb_files/code/logic-lynx-state-machine/build/cluon-msc /home/lynx/bbb_files/code/logic-lynx-state-machine/build/cluon-complete.cpp -std=c++14 -pthread -D HAVE_CLUON_MSC

CMakeFiles/state-machine-core.dir/src/logic-state-machine.cpp.o: CMakeFiles/state-machine-core.dir/flags.make
CMakeFiles/state-machine-core.dir/src/logic-state-machine.cpp.o: ../src/logic-state-machine.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lynx/bbb_files/code/logic-lynx-state-machine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/state-machine-core.dir/src/logic-state-machine.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/state-machine-core.dir/src/logic-state-machine.cpp.o -c /home/lynx/bbb_files/code/logic-lynx-state-machine/src/logic-state-machine.cpp

CMakeFiles/state-machine-core.dir/src/logic-state-machine.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/state-machine-core.dir/src/logic-state-machine.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lynx/bbb_files/code/logic-lynx-state-machine/src/logic-state-machine.cpp > CMakeFiles/state-machine-core.dir/src/logic-state-machine.cpp.i

CMakeFiles/state-machine-core.dir/src/logic-state-machine.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/state-machine-core.dir/src/logic-state-machine.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lynx/bbb_files/code/logic-lynx-state-machine/src/logic-state-machine.cpp -o CMakeFiles/state-machine-core.dir/src/logic-state-machine.cpp.s

CMakeFiles/state-machine-core.dir/src/logic-state-machine.cpp.o.requires:

.PHONY : CMakeFiles/state-machine-core.dir/src/logic-state-machine.cpp.o.requires

CMakeFiles/state-machine-core.dir/src/logic-state-machine.cpp.o.provides: CMakeFiles/state-machine-core.dir/src/logic-state-machine.cpp.o.requires
	$(MAKE) -f CMakeFiles/state-machine-core.dir/build.make CMakeFiles/state-machine-core.dir/src/logic-state-machine.cpp.o.provides.build
.PHONY : CMakeFiles/state-machine-core.dir/src/logic-state-machine.cpp.o.provides

CMakeFiles/state-machine-core.dir/src/logic-state-machine.cpp.o.provides.build: CMakeFiles/state-machine-core.dir/src/logic-state-machine.cpp.o


CMakeFiles/state-machine-core.dir/opendlv-standard-message-set.cpp.o: CMakeFiles/state-machine-core.dir/flags.make
CMakeFiles/state-machine-core.dir/opendlv-standard-message-set.cpp.o: opendlv-standard-message-set.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lynx/bbb_files/code/logic-lynx-state-machine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/state-machine-core.dir/opendlv-standard-message-set.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/state-machine-core.dir/opendlv-standard-message-set.cpp.o -c /home/lynx/bbb_files/code/logic-lynx-state-machine/build/opendlv-standard-message-set.cpp

CMakeFiles/state-machine-core.dir/opendlv-standard-message-set.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/state-machine-core.dir/opendlv-standard-message-set.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lynx/bbb_files/code/logic-lynx-state-machine/build/opendlv-standard-message-set.cpp > CMakeFiles/state-machine-core.dir/opendlv-standard-message-set.cpp.i

CMakeFiles/state-machine-core.dir/opendlv-standard-message-set.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/state-machine-core.dir/opendlv-standard-message-set.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lynx/bbb_files/code/logic-lynx-state-machine/build/opendlv-standard-message-set.cpp -o CMakeFiles/state-machine-core.dir/opendlv-standard-message-set.cpp.s

CMakeFiles/state-machine-core.dir/opendlv-standard-message-set.cpp.o.requires:

.PHONY : CMakeFiles/state-machine-core.dir/opendlv-standard-message-set.cpp.o.requires

CMakeFiles/state-machine-core.dir/opendlv-standard-message-set.cpp.o.provides: CMakeFiles/state-machine-core.dir/opendlv-standard-message-set.cpp.o.requires
	$(MAKE) -f CMakeFiles/state-machine-core.dir/build.make CMakeFiles/state-machine-core.dir/opendlv-standard-message-set.cpp.o.provides.build
.PHONY : CMakeFiles/state-machine-core.dir/opendlv-standard-message-set.cpp.o.provides

CMakeFiles/state-machine-core.dir/opendlv-standard-message-set.cpp.o.provides.build: CMakeFiles/state-machine-core.dir/opendlv-standard-message-set.cpp.o


state-machine-core: CMakeFiles/state-machine-core.dir/src/logic-state-machine.cpp.o
state-machine-core: CMakeFiles/state-machine-core.dir/opendlv-standard-message-set.cpp.o
state-machine-core: CMakeFiles/state-machine-core.dir/build.make

.PHONY : state-machine-core

# Rule to build all files generated by this target.
CMakeFiles/state-machine-core.dir/build: state-machine-core

.PHONY : CMakeFiles/state-machine-core.dir/build

CMakeFiles/state-machine-core.dir/requires: CMakeFiles/state-machine-core.dir/src/logic-state-machine.cpp.o.requires
CMakeFiles/state-machine-core.dir/requires: CMakeFiles/state-machine-core.dir/opendlv-standard-message-set.cpp.o.requires

.PHONY : CMakeFiles/state-machine-core.dir/requires

CMakeFiles/state-machine-core.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/state-machine-core.dir/cmake_clean.cmake
.PHONY : CMakeFiles/state-machine-core.dir/clean

CMakeFiles/state-machine-core.dir/depend: opendlv-standard-message-set.cpp
CMakeFiles/state-machine-core.dir/depend: cluon-msc
	cd /home/lynx/bbb_files/code/logic-lynx-state-machine/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lynx/bbb_files/code/logic-lynx-state-machine /home/lynx/bbb_files/code/logic-lynx-state-machine /home/lynx/bbb_files/code/logic-lynx-state-machine/build /home/lynx/bbb_files/code/logic-lynx-state-machine/build /home/lynx/bbb_files/code/logic-lynx-state-machine/build/CMakeFiles/state-machine-core.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/state-machine-core.dir/depend

