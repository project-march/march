## Some fancy cmake macros courtesy of the folks at Houston Mechatronics ##

macro(_lint_create_targets)
    # Create the master "lint" target if it doesn't exist yet.
    if(NOT TARGET lint)
        add_custom_target(lint)
    endif()

    # Create the "lint_pkgname" target if it doesn't exist yet. Doing this
    # with a check means that multiple linters can share the same target.
    if(NOT TARGET lint_${PROJECT_NAME})
        add_custom_target(lint_${PROJECT_NAME})
        add_dependencies(lint lint_${PROJECT_NAME})
    endif()

    # Set up lint output directory
    set(LINT_OUTPUT_DIR ${CMAKE_BINARY_DIR}/static_analysis)
    if(NOT TARGET lint_output_dir_${PROJECT_NAME})
        add_custom_target(lint_output_dir_${PROJECT_NAME}
                COMMAND ${CMAKE_COMMAND} -E make_directory ${LINT_OUTPUT_DIR})
    endif()
endmacro()

# Run cppcheck on specified files
#
# :param INCLUDE_DIRS: include directories
# :param SRCS: a non-empty list of files to process.
# :param OPTS: extra options to cppcheck
# :type string
function(lint_cpp)
    set(run_check TRUE)
    if(NOT CPPCHECK_PATH)
        find_program(CPPCHECK_PATH cppcheck)
        set(run_check TRUE)
        if(NOT CPPCHECK_PATH)
            set(run_check FALSE)
            message(WARNING "cppcheck not found! C++ code analysis target will NOT be built. To fix this, install cppcheck.")
        endif() # NOT CPPCHECK_PATH
    endif()

    if(NOT CPPCHECK_JUNIT_PATH)
        find_program(CPPCHECK_JUNIT_PATH cppcheck_junit)
        if(NOT CPPCHECK_JUNIT_PATH)
            set(run_check FALSE)
            message(WARNING "cppcheck_junit not found! C++ code analysis target will NOT be built. To fix this, 'pip install cppcheck-junit.'")
        endif() # NOT CPPCHECK_JUNIT_PATH
    endif()

    if(${run_check})
        _lint_create_targets()

        # Parses ARGN and sets lint_cpp_INCLUDE_DIRS and lint_cpp_SRCS
        set(multiValueArgs INCLUDE_DIRS SRCS OPTS)
        cmake_parse_arguments(lint_cpp "" "" "${multiValueArgs}" ${ARGN})

        # Must convert the (possible) list of include directories into:
        #  "-I include/dir/one -I include/dir/two .."
        set(cppcheck_include_dirs "")
        foreach(include_dir ${lint_cpp_INCLUDE_DIRS})
            set(cppcheck_include_dirs ${cppcheck_include_dirs} -I ${include_dir})
        endforeach()

        # Build rest of the cppcheck/cppcheck_junit command
        set(CPPCHECK_OUTPUT_FILE "${LINT_OUTPUT_DIR}/cppcheck_${PROJECT_NAME}.xml")

        # Add target and link to project lint target
        #   NOTE: add_custom_target is EXTREMELY finicky with the COMMAND specification. You
        #         CANNOT pass in a CMake string variable (aka ${LINT_CPP_CMD}). If you do
        #         it will insert quotes with the command is excuted.
        add_custom_target(lint_cpp_${PROJECT_NAME}
                COMMAND ${CPPCHECK_PATH} ${lint_cpp_OPTS} --platform=unix64 --xml-version=2 --std=c11 ${cppcheck_include_dirs} ${lint_cpp_SRCS} 2> ${CPPCHECK_OUTPUT_FILE} VERBATIM
                DEPENDS lint_output_dir_${PROJECT_NAME}
                WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
        add_dependencies(lint_${PROJECT_NAME} lint_cpp_${PROJECT_NAME})

        # Hook into rostest
        set(CPPCHECK_JUNIT_OUTPUT_FILE "lint-cppcheck-${PROJECT_NAME}.xml")
        set(CPPCHECK_JUNIT_OUTPUT_PATH "${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME}")
        # NOTE: This seems dumb. But catkin expects results to be located in the above path
        #       However, it does not create that path until it tries to find your files
        #       and they don't exist, then it drops a MISSING-XXXX.xml file in there.
        file(MAKE_DIRECTORY ${CPPCHECK_JUNIT_OUTPUT_PATH})
        catkin_run_tests_target("lint" "cpp" "${CPPCHECK_JUNIT_OUTPUT_FILE}"
                COMMAND "${CPPCHECK_JUNIT_PATH} ${CPPCHECK_OUTPUT_FILE} ${CPPCHECK_JUNIT_OUTPUT_PATH}/${CPPCHECK_JUNIT_OUTPUT_FILE}"
                DEPENDENCIES lint_cpp_${PROJECT_NAME}
                WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
    endif()
endfunction()

# Run cppcheck on library source files
#
# :param INCLUDE_DIRS: include directories
# :param SRCS: a non-empty list of files to process.
# :type string
function(lint_library)
    # Parses ARGN and sets lint_cpp_INCLUDE_DIRS and lint_cpp_SRCS
    set(multiValueArgs INCLUDE_DIRS SRCS)
    cmake_parse_arguments(lint_library "" "" "${multiValueArgs}" ${ARGN})

    lint_cpp(INCLUDE_DIRS ${lint_library_INCLUDE_DIRS}
            SRCS ${lint_library_SRCS}
            OPTS --inline-suppr --enable=warning,performance,portability,style)
endfunction()
