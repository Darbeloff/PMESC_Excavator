# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/weitung/excavation_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/weitung/excavation_ws/build

# Utility rule file for _run_tests_opencv_apps_rostest_test-face_detection__gui_false.test.

# Include the progress variables for this target.
include opencv_apps/test/CMakeFiles/_run_tests_opencv_apps_rostest_test-face_detection__gui_false.test.dir/progress.make

opencv_apps/test/CMakeFiles/_run_tests_opencv_apps_rostest_test-face_detection__gui_false.test:
	cd /home/weitung/excavation_ws/build/opencv_apps/test && ../../catkin_generated/env_cached.sh /usr/bin/python /opt/ros/indigo/share/catkin/cmake/test/run_tests.py /home/weitung/excavation_ws/build/test_results/opencv_apps/rostest-test-face_detection__gui_false.xml /opt/ros/indigo/share/rostest/cmake/../../../bin/rostest\ --pkgdir=/home/weitung/excavation_ws/src/opencv_apps\ --package=opencv_apps\ --results-filename\ test-face_detection__gui_false.xml\ --results-base-dir\ "/home/weitung/excavation_ws/build/test_results"\ /home/weitung/excavation_ws/src/opencv_apps/test/test-face_detection.test\ gui:=false

_run_tests_opencv_apps_rostest_test-face_detection__gui_false.test: opencv_apps/test/CMakeFiles/_run_tests_opencv_apps_rostest_test-face_detection__gui_false.test
_run_tests_opencv_apps_rostest_test-face_detection__gui_false.test: opencv_apps/test/CMakeFiles/_run_tests_opencv_apps_rostest_test-face_detection__gui_false.test.dir/build.make
.PHONY : _run_tests_opencv_apps_rostest_test-face_detection__gui_false.test

# Rule to build all files generated by this target.
opencv_apps/test/CMakeFiles/_run_tests_opencv_apps_rostest_test-face_detection__gui_false.test.dir/build: _run_tests_opencv_apps_rostest_test-face_detection__gui_false.test
.PHONY : opencv_apps/test/CMakeFiles/_run_tests_opencv_apps_rostest_test-face_detection__gui_false.test.dir/build

opencv_apps/test/CMakeFiles/_run_tests_opencv_apps_rostest_test-face_detection__gui_false.test.dir/clean:
	cd /home/weitung/excavation_ws/build/opencv_apps/test && $(CMAKE_COMMAND) -P CMakeFiles/_run_tests_opencv_apps_rostest_test-face_detection__gui_false.test.dir/cmake_clean.cmake
.PHONY : opencv_apps/test/CMakeFiles/_run_tests_opencv_apps_rostest_test-face_detection__gui_false.test.dir/clean

opencv_apps/test/CMakeFiles/_run_tests_opencv_apps_rostest_test-face_detection__gui_false.test.dir/depend:
	cd /home/weitung/excavation_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/weitung/excavation_ws/src /home/weitung/excavation_ws/src/opencv_apps/test /home/weitung/excavation_ws/build /home/weitung/excavation_ws/build/opencv_apps/test /home/weitung/excavation_ws/build/opencv_apps/test/CMakeFiles/_run_tests_opencv_apps_rostest_test-face_detection__gui_false.test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : opencv_apps/test/CMakeFiles/_run_tests_opencv_apps_rostest_test-face_detection__gui_false.test.dir/depend

