# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /snap/clion/190/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /snap/clion/190/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/alumno/RoboticaAvanzada

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/alumno/RoboticaAvanzada

# Utility rule file for giraff_viewer_autogen.

# Include any custom commands dependencies for this target.
include src/CMakeFiles/giraff_viewer_autogen.dir/compiler_depend.make

# Include the progress variables for this target.
include src/CMakeFiles/giraff_viewer_autogen.dir/progress.make

src/CMakeFiles/giraff_viewer_autogen:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/alumno/RoboticaAvanzada/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic MOC and UIC for target giraff_viewer"
	cd /home/alumno/RoboticaAvanzada/src && /snap/clion/190/bin/cmake/linux/bin/cmake -E cmake_autogen /home/alumno/RoboticaAvanzada/src/CMakeFiles/giraff_viewer_autogen.dir/AutogenInfo.json ""

giraff_viewer_autogen: src/CMakeFiles/giraff_viewer_autogen
giraff_viewer_autogen: src/CMakeFiles/giraff_viewer_autogen.dir/build.make
.PHONY : giraff_viewer_autogen

# Rule to build all files generated by this target.
src/CMakeFiles/giraff_viewer_autogen.dir/build: giraff_viewer_autogen
.PHONY : src/CMakeFiles/giraff_viewer_autogen.dir/build

src/CMakeFiles/giraff_viewer_autogen.dir/clean:
	cd /home/alumno/RoboticaAvanzada/src && $(CMAKE_COMMAND) -P CMakeFiles/giraff_viewer_autogen.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/giraff_viewer_autogen.dir/clean

src/CMakeFiles/giraff_viewer_autogen.dir/depend:
	cd /home/alumno/RoboticaAvanzada && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alumno/RoboticaAvanzada /home/alumno/RoboticaAvanzada/src /home/alumno/RoboticaAvanzada /home/alumno/RoboticaAvanzada/src /home/alumno/RoboticaAvanzada/src/CMakeFiles/giraff_viewer_autogen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/giraff_viewer_autogen.dir/depend

