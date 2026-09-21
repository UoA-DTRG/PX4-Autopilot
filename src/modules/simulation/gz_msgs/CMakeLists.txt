############################################################################
#
#   Copyright (c) 2025 PX4 Development Team. All rights reserved.
#
############################################################################

# Find required packages
find_package(Protobuf)

if (Protobuf_FOUND)
    # Generate protobuf messages
    file(GLOB MSGS_PROTOS "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")
    PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${MSGS_PROTOS})

    # Create library
    add_library(px4_gz_msgs STATIC
        ${PROTO_SRCS}
        ${PROTO_HDRS}
    )

    target_include_directories(px4_gz_msgs PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
    # Treat protobuf/abseil headers as system headers so their warnings are not errors
    target_include_directories(px4_gz_msgs SYSTEM PUBLIC ${Protobuf_INCLUDE_DIRS})
    target_link_libraries(px4_gz_msgs PUBLIC ${PROTOBUF_LIBRARIES})
    # protobuf >= 22 pulls in abseil, which requires C++17 and trips PX4's
    # -Werror=double-promotion in its own headers.
    set_target_properties(px4_gz_msgs PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
    target_compile_options(px4_gz_msgs PRIVATE -Wno-error=double-promotion)
    if (Protobuf_VERSION VERSION_LESS "3.8")
        target_compile_options(px4_gz_msgs PRIVATE -Wno-error=float-equal)
    endif()

    # Export the binary dir for other modules
    set(PX4_GZ_MSGS_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
endif()
