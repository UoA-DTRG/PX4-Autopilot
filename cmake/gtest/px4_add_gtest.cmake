############################################################################
#
# Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

include(GoogleTest)

#=============================================================================
#
#	px4_add_unit_gtest
#
#	Adds a googletest unit test to the test_results target.
#
#	DISCOVER: register every TEST() as its own ctest entry (named Suite.Test),
#	so ctest lists and reports each test case instead of the whole binary.
#
function(px4_add_unit_gtest)
	# skip if unit testing is not configured
	if(BUILD_TESTING)
		# parse source file and library dependencies from arguments
		px4_parse_function_args(
			NAME px4_add_unit_gtest
			OPTIONS DISCOVER
			ONE_VALUE SRC
			MULTI_VALUE EXTRA_SRCS COMPILE_FLAGS INCLUDES LINKLIBS
			REQUIRED SRC
			ARGN ${ARGN})

		# infer test name from source filname
		get_filename_component(TESTNAME ${SRC} NAME_WE)
		string(REPLACE Test "" TESTNAME ${TESTNAME})
		set(TESTNAME unit-${TESTNAME})

		# build a binary for the unit test
		add_executable(${TESTNAME} EXCLUDE_FROM_ALL ${SRC} ${EXTRA_SRCS})

		# link the libary to test and gtest
		target_link_libraries(${TESTNAME} ${LINKLIBS} gtest_main)

		if(COMPILE_FLAGS)
			target_compile_options(${TESTNAME} PRIVATE ${COMPILE_FLAGS})
		endif()

		if(INCLUDES)
			target_include_directories(${TESTNAME} PRIVATE ${INCLUDES})
		endif()

		# add the test to the ctest plan
		if(DISCOVER)
			gtest_discover_tests(${TESTNAME} WORKING_DIRECTORY ${PX4_BINARY_DIR})
		else()
			add_test(NAME ${TESTNAME}
			         COMMAND ${TESTNAME}
			         WORKING_DIRECTORY ${PX4_BINARY_DIR})
		endif()

		# attach it to the unit test target
		add_dependencies(test_results ${TESTNAME})
	endif()
endfunction()

function(px4_add_functional_gtest)
	# skip if unit testing is not configured
	if(BUILD_TESTING)
		# parse source file and library dependencies from arguments
		px4_parse_function_args(
			NAME px4_add_functional_gtest
			OPTIONS DISCOVER
			ONE_VALUE SRC
			MULTI_VALUE EXTRA_SRCS COMPILE_FLAGS INCLUDES LINKLIBS
			REQUIRED SRC
			ARGN ${ARGN})

		# infer test name from source filname
		get_filename_component(TESTNAME ${SRC} NAME_WE)
		string(REPLACE Test "" TESTNAME ${TESTNAME})
		set(TESTNAME functional-${TESTNAME})

		# build a binary for the unit test
		add_executable(${TESTNAME} EXCLUDE_FROM_ALL ${SRC} ${EXTRA_SRCS})

		# link the libary to test and gtest
		target_link_libraries(${TESTNAME} ${LINKLIBS} gtest_functional_main
		                                              px4_layer
		                                              px4_platform
		                                              uORB
		                                              systemlib
		                                              cdev
		                                              px4_work_queue
		                                              px4_daemon
		                                              work_queue
		                                              parameters
		                                              events
		                                              perf
		                                              tinybson
		                                              uorb_msgs
		                                              test_stubs)  #put test_stubs last

		if(COMPILE_FLAGS)
			target_compile_options(${TESTNAME} PRIVATE ${COMPILE_FLAGS})
		endif()

		if(INCLUDES)
			target_include_directories(${TESTNAME} PRIVATE ${INCLUDES})
		endif()

		# add the test to the ctest plan
		if(DISCOVER)
			# runs each test case in its own process
			gtest_discover_tests(${TESTNAME} WORKING_DIRECTORY ${PX4_BINARY_DIR})
		else()
			add_test(NAME ${TESTNAME}
			         # functional tests need to run in a new process for each test,
			         # since they set up and tear down system components
			         COMMAND ${PX4_BINARY_DIR}/${TESTNAME})
		endif()

		# attach it to the unit test target
		add_dependencies(test_results ${TESTNAME})
	endif()
endfunction()
