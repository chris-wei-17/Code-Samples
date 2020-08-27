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
CMAKE_SOURCE_DIR = /home/jetson1/Racecar/racecar_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jetson1/Racecar/racecar_ws/build

# Utility rule file for openni2_camera_gencfg.

# Include the progress variables for this target.
include openni2_camera/openni2_camera/CMakeFiles/openni2_camera_gencfg.dir/progress.make

openni2_camera/openni2_camera/CMakeFiles/openni2_camera_gencfg: /home/jetson1/Racecar/racecar_ws/devel/include/openni2_camera/OpenNI2Config.h
openni2_camera/openni2_camera/CMakeFiles/openni2_camera_gencfg: /home/jetson1/Racecar/racecar_ws/devel/lib/python2.7/dist-packages/openni2_camera/cfg/OpenNI2Config.py


/home/jetson1/Racecar/racecar_ws/devel/include/openni2_camera/OpenNI2Config.h: /home/jetson1/Racecar/racecar_ws/src/openni2_camera/openni2_camera/cfg/OpenNI2.cfg
/home/jetson1/Racecar/racecar_ws/devel/include/openni2_camera/OpenNI2Config.h: /opt/ros/melodic/share/dynamic_reconfigure/templates/ConfigType.py.template
/home/jetson1/Racecar/racecar_ws/devel/include/openni2_camera/OpenNI2Config.h: /opt/ros/melodic/share/dynamic_reconfigure/templates/ConfigType.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/jetson1/Racecar/racecar_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating dynamic reconfigure files from cfg/OpenNI2.cfg: /home/jetson1/Racecar/racecar_ws/devel/include/openni2_camera/OpenNI2Config.h /home/jetson1/Racecar/racecar_ws/devel/lib/python2.7/dist-packages/openni2_camera/cfg/OpenNI2Config.py"
	cd /home/jetson1/Racecar/racecar_ws/build/openni2_camera/openni2_camera && ../../catkin_generated/env_cached.sh /home/jetson1/Racecar/racecar_ws/build/openni2_camera/openni2_camera/setup_custom_pythonpath.sh /home/jetson1/Racecar/racecar_ws/src/openni2_camera/openni2_camera/cfg/OpenNI2.cfg /opt/ros/melodic/share/dynamic_reconfigure/cmake/.. /home/jetson1/Racecar/racecar_ws/devel/share/openni2_camera /home/jetson1/Racecar/racecar_ws/devel/include/openni2_camera /home/jetson1/Racecar/racecar_ws/devel/lib/python2.7/dist-packages/openni2_camera

/home/jetson1/Racecar/racecar_ws/devel/share/openni2_camera/docs/OpenNI2Config.dox: /home/jetson1/Racecar/racecar_ws/devel/include/openni2_camera/OpenNI2Config.h
	@$(CMAKE_COMMAND) -E touch_nocreate /home/jetson1/Racecar/racecar_ws/devel/share/openni2_camera/docs/OpenNI2Config.dox

/home/jetson1/Racecar/racecar_ws/devel/share/openni2_camera/docs/OpenNI2Config-usage.dox: /home/jetson1/Racecar/racecar_ws/devel/include/openni2_camera/OpenNI2Config.h
	@$(CMAKE_COMMAND) -E touch_nocreate /home/jetson1/Racecar/racecar_ws/devel/share/openni2_camera/docs/OpenNI2Config-usage.dox

/home/jetson1/Racecar/racecar_ws/devel/lib/python2.7/dist-packages/openni2_camera/cfg/OpenNI2Config.py: /home/jetson1/Racecar/racecar_ws/devel/include/openni2_camera/OpenNI2Config.h
	@$(CMAKE_COMMAND) -E touch_nocreate /home/jetson1/Racecar/racecar_ws/devel/lib/python2.7/dist-packages/openni2_camera/cfg/OpenNI2Config.py

/home/jetson1/Racecar/racecar_ws/devel/share/openni2_camera/docs/OpenNI2Config.wikidoc: /home/jetson1/Racecar/racecar_ws/devel/include/openni2_camera/OpenNI2Config.h
	@$(CMAKE_COMMAND) -E touch_nocreate /home/jetson1/Racecar/racecar_ws/devel/share/openni2_camera/docs/OpenNI2Config.wikidoc

openni2_camera_gencfg: openni2_camera/openni2_camera/CMakeFiles/openni2_camera_gencfg
openni2_camera_gencfg: /home/jetson1/Racecar/racecar_ws/devel/include/openni2_camera/OpenNI2Config.h
openni2_camera_gencfg: /home/jetson1/Racecar/racecar_ws/devel/share/openni2_camera/docs/OpenNI2Config.dox
openni2_camera_gencfg: /home/jetson1/Racecar/racecar_ws/devel/share/openni2_camera/docs/OpenNI2Config-usage.dox
openni2_camera_gencfg: /home/jetson1/Racecar/racecar_ws/devel/lib/python2.7/dist-packages/openni2_camera/cfg/OpenNI2Config.py
openni2_camera_gencfg: /home/jetson1/Racecar/racecar_ws/devel/share/openni2_camera/docs/OpenNI2Config.wikidoc
openni2_camera_gencfg: openni2_camera/openni2_camera/CMakeFiles/openni2_camera_gencfg.dir/build.make

.PHONY : openni2_camera_gencfg

# Rule to build all files generated by this target.
openni2_camera/openni2_camera/CMakeFiles/openni2_camera_gencfg.dir/build: openni2_camera_gencfg

.PHONY : openni2_camera/openni2_camera/CMakeFiles/openni2_camera_gencfg.dir/build

openni2_camera/openni2_camera/CMakeFiles/openni2_camera_gencfg.dir/clean:
	cd /home/jetson1/Racecar/racecar_ws/build/openni2_camera/openni2_camera && $(CMAKE_COMMAND) -P CMakeFiles/openni2_camera_gencfg.dir/cmake_clean.cmake
.PHONY : openni2_camera/openni2_camera/CMakeFiles/openni2_camera_gencfg.dir/clean

openni2_camera/openni2_camera/CMakeFiles/openni2_camera_gencfg.dir/depend:
	cd /home/jetson1/Racecar/racecar_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jetson1/Racecar/racecar_ws/src /home/jetson1/Racecar/racecar_ws/src/openni2_camera/openni2_camera /home/jetson1/Racecar/racecar_ws/build /home/jetson1/Racecar/racecar_ws/build/openni2_camera/openni2_camera /home/jetson1/Racecar/racecar_ws/build/openni2_camera/openni2_camera/CMakeFiles/openni2_camera_gencfg.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : openni2_camera/openni2_camera/CMakeFiles/openni2_camera_gencfg.dir/depend

