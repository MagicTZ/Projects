# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/magictz/backup/cmake/t3

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/magictz/backup/cmake/t3/build

# Include any dependencies generated for this target.
include lib/CMakeFiles/t3.dir/depend.make

# Include the progress variables for this target.
include lib/CMakeFiles/t3.dir/progress.make

# Include the compile flags for this target's objects.
include lib/CMakeFiles/t3.dir/flags.make

lib/CMakeFiles/t3.dir/hello.cc.o: lib/CMakeFiles/t3.dir/flags.make
lib/CMakeFiles/t3.dir/hello.cc.o: ../lib/hello.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/magictz/backup/cmake/t3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object lib/CMakeFiles/t3.dir/hello.cc.o"
	cd /home/magictz/backup/cmake/t3/build/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/t3.dir/hello.cc.o -c /home/magictz/backup/cmake/t3/lib/hello.cc

lib/CMakeFiles/t3.dir/hello.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/t3.dir/hello.cc.i"
	cd /home/magictz/backup/cmake/t3/build/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/magictz/backup/cmake/t3/lib/hello.cc > CMakeFiles/t3.dir/hello.cc.i

lib/CMakeFiles/t3.dir/hello.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/t3.dir/hello.cc.s"
	cd /home/magictz/backup/cmake/t3/build/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/magictz/backup/cmake/t3/lib/hello.cc -o CMakeFiles/t3.dir/hello.cc.s

# Object files for target t3
t3_OBJECTS = \
"CMakeFiles/t3.dir/hello.cc.o"

# External object files for target t3
t3_EXTERNAL_OBJECTS =

lib/t3: lib/CMakeFiles/t3.dir/hello.cc.o
lib/t3: lib/CMakeFiles/t3.dir/build.make
lib/t3: lib/CMakeFiles/t3.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/magictz/backup/cmake/t3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable t3"
	cd /home/magictz/backup/cmake/t3/build/lib && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/t3.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lib/CMakeFiles/t3.dir/build: lib/t3

.PHONY : lib/CMakeFiles/t3.dir/build

lib/CMakeFiles/t3.dir/clean:
	cd /home/magictz/backup/cmake/t3/build/lib && $(CMAKE_COMMAND) -P CMakeFiles/t3.dir/cmake_clean.cmake
.PHONY : lib/CMakeFiles/t3.dir/clean

lib/CMakeFiles/t3.dir/depend:
	cd /home/magictz/backup/cmake/t3/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/magictz/backup/cmake/t3 /home/magictz/backup/cmake/t3/lib /home/magictz/backup/cmake/t3/build /home/magictz/backup/cmake/t3/build/lib /home/magictz/backup/cmake/t3/build/lib/CMakeFiles/t3.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lib/CMakeFiles/t3.dir/depend
