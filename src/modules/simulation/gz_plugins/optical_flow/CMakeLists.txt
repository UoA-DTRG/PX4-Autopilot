############################################################################
#
#   Copyright (c) 2025 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

project(OpticalFlowSystem)

# PX4-OpticalFlow's klt_feature_tracker still includes OpenCV 4 headers
# (opencv2/core/types_c.h, opencv2/features2d/features2d.hpp) that OpenCV 5 removed, so
# this plugin needs an OpenCV 4.x. Homebrew's default opencv is 5.x but it also packages
# opencv@4; that one is keg-only, so CMake only finds it when pointed at it.
if(APPLE AND NOT OpenCV_DIR)
    find_program(BREW_EXECUTABLE brew)

    if(BREW_EXECUTABLE)
        execute_process(
            COMMAND ${BREW_EXECUTABLE} --prefix opencv@4
            OUTPUT_VARIABLE OPENCV4_BREW_PREFIX
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
        )

        if(OPENCV4_BREW_PREFIX AND EXISTS "${OPENCV4_BREW_PREFIX}/lib/cmake/opencv4")
            set(OpenCV_DIR "${OPENCV4_BREW_PREFIX}/lib/cmake/opencv4")
        endif()
    endif()
endif()

# Skip the plugin rather than failing the build when no usable OpenCV is present, the same
# way the gstreamer plugin is skipped when GStreamer is missing. The parent CMakeLists only
# adds this plugin to px4_gz_plugins when the target actually got defined.
find_package(OpenCV QUIET)

if(NOT OpenCV_FOUND)
    message(STATUS "gz optical flow plugin disabled: OpenCV not found")
    return()
endif()

if(NOT OpenCV_VERSION VERSION_LESS 5)
    if(APPLE)
        message(STATUS "gz optical flow plugin disabled: needs OpenCV 4, found "
            "${OpenCV_VERSION} (try: brew install opencv@4)")
    else()
        message(STATUS "gz optical flow plugin disabled: needs OpenCV 4, found ${OpenCV_VERSION}")
    endif()

    return()
endif()

# Include the OpticalFlow external dependency
include(${CMAKE_CURRENT_SOURCE_DIR}/optical_flow.cmake)

# Find OpenCV
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

add_library(${PROJECT_NAME} SHARED
    OpticalFlowSensor.cpp
    OpticalFlowSystem.cpp
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC px4_gz_msgs
    PUBLIC gz-sensors${gz-sensors_VERSION_MAJOR}::gz-sensors${gz-sensors_VERSION_MAJOR}
    PUBLIC gz-plugin${gz-plugin_VERSION_MAJOR}::gz-plugin${gz-plugin_VERSION_MAJOR}
    PUBLIC gz-sim${gz-sim_VERSION_MAJOR}::gz-sim${gz-sim_VERSION_MAJOR}
    PUBLIC gz-transport${gz-transport_VERSION_MAJOR}::gz-transport${gz-transport_VERSION_MAJOR}
    PUBLIC ${OpenCV_LIBS}
    PUBLIC ${OpticalFlow_LIBS}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
    PUBLIC ${OpenCV_INCLUDE_DIRS}
    PUBLIC ${OpticalFlow_INCLUDE_DIRS}
    PUBLIC px4_gz_msgs
)

# PX4-OpticalFlow is built with the default macOS install name (@rpath/libOpticalFlow.dylib),
# so the plugin needs an rpath pointing at the external project's install tree or dlopen
# fails at runtime with "Library not loaded: @rpath/libOpticalFlow.dylib".
if(APPLE)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        BUILD_RPATH "${OpticalFlow_LIBRARY_DIRS}"
    )
endif()

add_dependencies(${PROJECT_NAME} OpticalFlow)
