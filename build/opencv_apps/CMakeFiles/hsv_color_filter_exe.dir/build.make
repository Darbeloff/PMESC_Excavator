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

# Include any dependencies generated for this target.
include opencv_apps/CMakeFiles/hsv_color_filter_exe.dir/depend.make

# Include the progress variables for this target.
include opencv_apps/CMakeFiles/hsv_color_filter_exe.dir/progress.make

# Include the compile flags for this target's objects.
include opencv_apps/CMakeFiles/hsv_color_filter_exe.dir/flags.make

opencv_apps/CMakeFiles/hsv_color_filter_exe.dir/hsv_color_filter.cpp.o: opencv_apps/CMakeFiles/hsv_color_filter_exe.dir/flags.make
opencv_apps/CMakeFiles/hsv_color_filter_exe.dir/hsv_color_filter.cpp.o: opencv_apps/hsv_color_filter.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/weitung/excavation_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object opencv_apps/CMakeFiles/hsv_color_filter_exe.dir/hsv_color_filter.cpp.o"
	cd /home/weitung/excavation_ws/build/opencv_apps && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/hsv_color_filter_exe.dir/hsv_color_filter.cpp.o -c /home/weitung/excavation_ws/build/opencv_apps/hsv_color_filter.cpp

opencv_apps/CMakeFiles/hsv_color_filter_exe.dir/hsv_color_filter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hsv_color_filter_exe.dir/hsv_color_filter.cpp.i"
	cd /home/weitung/excavation_ws/build/opencv_apps && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/weitung/excavation_ws/build/opencv_apps/hsv_color_filter.cpp > CMakeFiles/hsv_color_filter_exe.dir/hsv_color_filter.cpp.i

opencv_apps/CMakeFiles/hsv_color_filter_exe.dir/hsv_color_filter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hsv_color_filter_exe.dir/hsv_color_filter.cpp.s"
	cd /home/weitung/excavation_ws/build/opencv_apps && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/weitung/excavation_ws/build/opencv_apps/hsv_color_filter.cpp -o CMakeFiles/hsv_color_filter_exe.dir/hsv_color_filter.cpp.s

opencv_apps/CMakeFiles/hsv_color_filter_exe.dir/hsv_color_filter.cpp.o.requires:
.PHONY : opencv_apps/CMakeFiles/hsv_color_filter_exe.dir/hsv_color_filter.cpp.o.requires

opencv_apps/CMakeFiles/hsv_color_filter_exe.dir/hsv_color_filter.cpp.o.provides: opencv_apps/CMakeFiles/hsv_color_filter_exe.dir/hsv_color_filter.cpp.o.requires
	$(MAKE) -f opencv_apps/CMakeFiles/hsv_color_filter_exe.dir/build.make opencv_apps/CMakeFiles/hsv_color_filter_exe.dir/hsv_color_filter.cpp.o.provides.build
.PHONY : opencv_apps/CMakeFiles/hsv_color_filter_exe.dir/hsv_color_filter.cpp.o.provides

opencv_apps/CMakeFiles/hsv_color_filter_exe.dir/hsv_color_filter.cpp.o.provides.build: opencv_apps/CMakeFiles/hsv_color_filter_exe.dir/hsv_color_filter.cpp.o

# Object files for target hsv_color_filter_exe
hsv_color_filter_exe_OBJECTS = \
"CMakeFiles/hsv_color_filter_exe.dir/hsv_color_filter.cpp.o"

# External object files for target hsv_color_filter_exe
hsv_color_filter_exe_EXTERNAL_OBJECTS =

/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: opencv_apps/CMakeFiles/hsv_color_filter_exe.dir/hsv_color_filter.cpp.o
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: opencv_apps/CMakeFiles/hsv_color_filter_exe.dir/build.make
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /opt/ros/indigo/lib/libcv_bridge.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /opt/ros/indigo/lib/libdynamic_reconfigure_config_init_mutex.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /opt/ros/indigo/lib/libimage_transport.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /opt/ros/indigo/lib/libmessage_filters.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /opt/ros/indigo/lib/libnodeletlib.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /opt/ros/indigo/lib/libbondcpp.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /opt/ros/indigo/lib/libclass_loader.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/libPocoFoundation.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libdl.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /opt/ros/indigo/lib/libroslib.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /opt/ros/indigo/lib/librospack.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /opt/ros/indigo/lib/libroscpp.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /opt/ros/indigo/lib/librosconsole.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/liblog4cxx.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /opt/ros/indigo/lib/librostime.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /opt/ros/indigo/lib/libcpp_common.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
/home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter: opencv_apps/CMakeFiles/hsv_color_filter_exe.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter"
	cd /home/weitung/excavation_ws/build/opencv_apps && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/hsv_color_filter_exe.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
opencv_apps/CMakeFiles/hsv_color_filter_exe.dir/build: /home/weitung/excavation_ws/devel/lib/opencv_apps/hsv_color_filter
.PHONY : opencv_apps/CMakeFiles/hsv_color_filter_exe.dir/build

opencv_apps/CMakeFiles/hsv_color_filter_exe.dir/requires: opencv_apps/CMakeFiles/hsv_color_filter_exe.dir/hsv_color_filter.cpp.o.requires
.PHONY : opencv_apps/CMakeFiles/hsv_color_filter_exe.dir/requires

opencv_apps/CMakeFiles/hsv_color_filter_exe.dir/clean:
	cd /home/weitung/excavation_ws/build/opencv_apps && $(CMAKE_COMMAND) -P CMakeFiles/hsv_color_filter_exe.dir/cmake_clean.cmake
.PHONY : opencv_apps/CMakeFiles/hsv_color_filter_exe.dir/clean

opencv_apps/CMakeFiles/hsv_color_filter_exe.dir/depend:
	cd /home/weitung/excavation_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/weitung/excavation_ws/src /home/weitung/excavation_ws/src/opencv_apps /home/weitung/excavation_ws/build /home/weitung/excavation_ws/build/opencv_apps /home/weitung/excavation_ws/build/opencv_apps/CMakeFiles/hsv_color_filter_exe.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : opencv_apps/CMakeFiles/hsv_color_filter_exe.dir/depend

