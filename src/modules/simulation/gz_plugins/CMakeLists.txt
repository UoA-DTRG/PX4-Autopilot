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

# Homebrew's qt@5 is keg-only, so CMake cannot find it on its own. gz-gui (pulled in
# by gz-sim below) requires Qt5, and without it configuration dies at generate time
# with "The link interface of target gz-gui8::gz-gui8 contains Qt5::Core".
if(APPLE AND NOT Qt5_DIR)
    find_program(BREW_EXECUTABLE brew)

    if(BREW_EXECUTABLE)
        execute_process(
            COMMAND ${BREW_EXECUTABLE} --prefix qt@5
            OUTPUT_VARIABLE QT5_BREW_PREFIX
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
        )

        if(QT5_BREW_PREFIX AND EXISTS "${QT5_BREW_PREFIX}/lib/cmake/Qt5")
            list(APPEND CMAKE_PREFIX_PATH "${QT5_BREW_PREFIX}")
        endif()
    endif()

    if(NOT CMAKE_PREFIX_PATH MATCHES "Qt5|qt@5" AND NOT DEFINED ENV{CMAKE_PREFIX_PATH})
        message(STATUS
            "Qt5 not located automatically; if configuring fails on gz-gui, "
            "install it (brew install qt@5) or set CMAKE_PREFIX_PATH to its prefix")
    endif()
endif()

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
    # optical_flow resolves its own OpenCV and skips itself when there is no usable one,
    # so all OpenCV lookup stays inside that directory (its imported targets must not leak
    # into this scope, or sibling plugins would link a mix of OpenCV versions).
    add_subdirectory(optical_flow)

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
