# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

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
CMAKE_SOURCE_DIR = /tmp/tmp.2l8OOBF7z1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /tmp/tmp.2l8OOBF7z1/cmake-build-debug-sw

# Include any dependencies generated for this target.
include CMakeFiles/Sw.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Sw.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Sw.dir/flags.make

CMakeFiles/Sw.dir/src/main.cpp.o: CMakeFiles/Sw.dir/flags.make
CMakeFiles/Sw.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/tmp/tmp.2l8OOBF7z1/cmake-build-debug-sw/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Sw.dir/src/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Sw.dir/src/main.cpp.o -c /tmp/tmp.2l8OOBF7z1/src/main.cpp

CMakeFiles/Sw.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Sw.dir/src/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /tmp/tmp.2l8OOBF7z1/src/main.cpp > CMakeFiles/Sw.dir/src/main.cpp.i

CMakeFiles/Sw.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Sw.dir/src/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /tmp/tmp.2l8OOBF7z1/src/main.cpp -o CMakeFiles/Sw.dir/src/main.cpp.s

CMakeFiles/Sw.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/Sw.dir/src/main.cpp.o.requires

CMakeFiles/Sw.dir/src/main.cpp.o.provides: CMakeFiles/Sw.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/Sw.dir/build.make CMakeFiles/Sw.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/Sw.dir/src/main.cpp.o.provides

CMakeFiles/Sw.dir/src/main.cpp.o.provides.build: CMakeFiles/Sw.dir/src/main.cpp.o


# Object files for target Sw
Sw_OBJECTS = \
"CMakeFiles/Sw.dir/src/main.cpp.o"

# External object files for target Sw
Sw_EXTERNAL_OBJECTS =

Sw: CMakeFiles/Sw.dir/src/main.cpp.o
Sw: CMakeFiles/Sw.dir/build.make
Sw: libs/bpc_prp_opnecv_lib/libbpc_prp_opencv_lib.a
Sw: /usr/local/lib/libopencv_gapi.so.4.0.0
Sw: /usr/local/lib/libopencv_stitching.so.4.0.0
Sw: /usr/local/lib/libopencv_aruco.so.4.0.0
Sw: /usr/local/lib/libopencv_bgsegm.so.4.0.0
Sw: /usr/local/lib/libopencv_bioinspired.so.4.0.0
Sw: /usr/local/lib/libopencv_ccalib.so.4.0.0
Sw: /usr/local/lib/libopencv_dnn_objdetect.so.4.0.0
Sw: /usr/local/lib/libopencv_dpm.so.4.0.0
Sw: /usr/local/lib/libopencv_face.so.4.0.0
Sw: /usr/local/lib/libopencv_fuzzy.so.4.0.0
Sw: /usr/local/lib/libopencv_hfs.so.4.0.0
Sw: /usr/local/lib/libopencv_img_hash.so.4.0.0
Sw: /usr/local/lib/libopencv_line_descriptor.so.4.0.0
Sw: /usr/local/lib/libopencv_reg.so.4.0.0
Sw: /usr/local/lib/libopencv_rgbd.so.4.0.0
Sw: /usr/local/lib/libopencv_saliency.so.4.0.0
Sw: /usr/local/lib/libopencv_stereo.so.4.0.0
Sw: /usr/local/lib/libopencv_structured_light.so.4.0.0
Sw: /usr/local/lib/libopencv_phase_unwrapping.so.4.0.0
Sw: /usr/local/lib/libopencv_superres.so.4.0.0
Sw: /usr/local/lib/libopencv_optflow.so.4.0.0
Sw: /usr/local/lib/libopencv_surface_matching.so.4.0.0
Sw: /usr/local/lib/libopencv_tracking.so.4.0.0
Sw: /usr/local/lib/libopencv_datasets.so.4.0.0
Sw: /usr/local/lib/libopencv_plot.so.4.0.0
Sw: /usr/local/lib/libopencv_text.so.4.0.0
Sw: /usr/local/lib/libopencv_dnn.so.4.0.0
Sw: /usr/local/lib/libopencv_videostab.so.4.0.0
Sw: /usr/local/lib/libopencv_photo.so.4.0.0
Sw: /usr/local/lib/libopencv_video.so.4.0.0
Sw: /usr/local/lib/libopencv_xfeatures2d.so.4.0.0
Sw: /usr/local/lib/libopencv_ml.so.4.0.0
Sw: /usr/local/lib/libopencv_shape.so.4.0.0
Sw: /usr/local/lib/libopencv_ximgproc.so.4.0.0
Sw: /usr/local/lib/libopencv_xobjdetect.so.4.0.0
Sw: /usr/local/lib/libopencv_objdetect.so.4.0.0
Sw: /usr/local/lib/libopencv_calib3d.so.4.0.0
Sw: /usr/local/lib/libopencv_features2d.so.4.0.0
Sw: /usr/local/lib/libopencv_flann.so.4.0.0
Sw: /usr/local/lib/libopencv_highgui.so.4.0.0
Sw: /usr/local/lib/libopencv_videoio.so.4.0.0
Sw: /usr/local/lib/libopencv_imgcodecs.so.4.0.0
Sw: /usr/local/lib/libopencv_xphoto.so.4.0.0
Sw: /usr/local/lib/libopencv_imgproc.so.4.0.0
Sw: /usr/local/lib/libopencv_core.so.4.0.0
Sw: CMakeFiles/Sw.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/tmp/tmp.2l8OOBF7z1/cmake-build-debug-sw/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable Sw"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Sw.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Sw.dir/build: Sw

.PHONY : CMakeFiles/Sw.dir/build

CMakeFiles/Sw.dir/requires: CMakeFiles/Sw.dir/src/main.cpp.o.requires

.PHONY : CMakeFiles/Sw.dir/requires

CMakeFiles/Sw.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Sw.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Sw.dir/clean

CMakeFiles/Sw.dir/depend:
	cd /tmp/tmp.2l8OOBF7z1/cmake-build-debug-sw && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /tmp/tmp.2l8OOBF7z1 /tmp/tmp.2l8OOBF7z1 /tmp/tmp.2l8OOBF7z1/cmake-build-debug-sw /tmp/tmp.2l8OOBF7z1/cmake-build-debug-sw /tmp/tmp.2l8OOBF7z1/cmake-build-debug-sw/CMakeFiles/Sw.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Sw.dir/depend

