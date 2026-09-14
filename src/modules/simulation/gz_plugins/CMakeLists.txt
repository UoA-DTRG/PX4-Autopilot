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

project(px4_gz_plugins)

if(NOT DEFINED ENV{GZ_DISTRO} OR NOT "$ENV{GZ_DISTRO}"  STREQUAL "harmonic")
    find_package(gz-transport NAMES gz-transport14 gz-transport13)
    find_package(gz-sim NAMES gz-sim9 gz-sim8)
    find_package(gz-sensors NAMES gz-sensors9 gz-sensors8)
    find_package(gz-plugin NAMES gz-plugin3 gz-plugin2 COMPONENTS register)
else()
    find_package(gz-transport NAMES gz-transport13)
    find_package(gz-sim NAMES gz-sim8)
    find_package(gz-sensors NAMES gz-sensors8)
    find_package(gz-plugin NAMES gz-plugin2 COMPONENTS register)
endif()

if (gz-transport_FOUND)
    # Create a flat output directory for all plugin libraries
    set(PX4_GZ_PLUGIN_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}" CACHE PATH "Directory for all Gazebo plugin libraries")
    file(MAKE_DIRECTORY ${PX4_GZ_PLUGIN_OUTPUT_DIR})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PX4_GZ_PLUGIN_OUTPUT_DIR})

    # Add our plugins as subdirectories
    # PX4-OpticalFlow hardcodes a Linux .so library name and requires OpenCV < 5,
    # so the optical flow plugin cannot be built on macOS.
    if (NOT APPLE)
        add_subdirectory(optical_flow)
    endif()

    add_subdirectory(template_plugin)
    add_subdirectory(gstreamer)
    add_subdirectory(moving_platform_controller)

    # Add an alias target for each plugin
    set(px4_gz_plugin_targets MovingPlatformController TemplatePlugin)

    if (TARGET OpticalFlowSystem)
        list(APPEND px4_gz_plugin_targets OpticalFlowSystem)
    endif()

    if (TARGET GstCameraSystem)
        list(APPEND px4_gz_plugin_targets GstCameraSystem)
    endif()

    add_custom_target(px4_gz_plugins ALL DEPENDS ${px4_gz_plugin_targets})
endif()
