####
# Based on cmake module found here: https://github.com/bilke/cmake-modules/blob/master/CodeCoverage.cmake
# Code Coverage (lcov) reporting targets and functions.
# Add project targets to coverage with `coverage_add_target()`
#
# Generate coverage report by calling the `run_coverage` target.
#
# When compiling you must use -DCMAKE_BUILD_TYPE=Coverage
####

# Set up program paths and compiler settings

find_program(GCOV_PATH gcov)
find_program(LCOV_PATH lcov)
find_program(GENHTML_PATH genhtml)

set(run TRUE)

IF(NOT GCOV_PATH OR NOT LCOV_PATH OR NOT GENHTML_PATH)
    set(run FALSE)
    MESSAGE(WARNING "To run code coverage, you must have gcov, lcov, and genhtml! One or all of them were not found...")
ENDIF()

set(CMAKE_CXX_FLAGS_COVERAGE
    "-g -O0 --coverage -fprofile-arcs -ftest-coverage"
    CACHE STRING "Flags used by the C++ compiler during coverage builds."
    FORCE
)
set(CMAKE_C_FLAGS_COVERAGE
    "-g -O0 --coverage -fprofile-arcs -ftest-coverage"
    CACHE STRING "Flags used by the C compiler during coverage builds."
    FORCE
)
set(CMAKE_EXE_LINKER_FLAGS_COVERAGE
    ""
    CACHE STRING "Flags used for linking binaries during coverage builds."
    FORCE
)
set(CMAKE_SHARED_LINKER_FLAGS_COVERAGE
    ""
    CACHE STRING "Flags used by the shared libraries linker during coverage builds."
    FORCE
)
mark_as_advanced(
    CMAKE_CXX_FLAGS_COVERAGE
    CMAKE_C_FLAGS_COVERAGE
    CMAKE_EXE_LINKER_FLAGS_COVERAGE
    CMAKE_SHARED_LINKER_FLAGS_COVERAGE
)

# Set up coverage targets
set(COVERAGE_DIR ${CMAKE_BINARY_DIR}/coverage)

add_custom_target(${PROJECT_NAME}_coverage_dir
    COMMAND ${CMAKE_COMMAND} -E make_directory ${COVERAGE_DIR}
)

add_custom_target(${PROJECT_NAME}_coverage_prep
    COMMAND ${LCOV_PATH} --quiet --directory ${CMAKE_BINARY_DIR} --zerocounters

    WORKING_DIRECTORY ${COVERAGE_DIR}
    DEPENDS ${PROJECT_NAME}_coverage_dir
)

function(coverage_add_target tgt)
    if(${run})
        add_dependencies(${tgt} ${PROJECT_NAME}_coverage_prep)

        add_custom_target(run_coverage_${PROJECT_NAME}
        COMMAND ${LCOV_PATH} --directory ${CMAKE_BINARY_DIR} --base-directory ${PROJECT_SOURCE_DIR} --capture --output-file ${PROJECT_NAME}.info
        COMMAND ${LCOV_PATH} --extract ${PROJECT_NAME}.info '${PROJECT_SOURCE_DIR}/*' --output-file ${PROJECT_NAME}.info.cleaned
        COMMAND ${GENHTML_PATH} -o ${COVERAGE_DIR} --show-details --legend ${PROJECT_NAME}.info.cleaned
        COMMAND ${CMAKE_COMMAND} -E remove ${PROJECT_NAME}.info ${PROJECT_NAME}.info.cleaned
        COMMAND ${CMAKE_COMMAND} -E echo "Coverage report found in: ${COVERAGE_DIR}"
     
        WORKING_DIRECTORY ${COVERAGE_DIR}
        DEPENDS ${tgt}
        )
    endif()

endfunction()
