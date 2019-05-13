
# clang-format is a style beautifier for c++ code
find_program(CLANG_FORMAT clang-format)
find_program(FIND find)
find_program(REMOVE rm)
find_program(COPY cp)

macro(format_code_target)
    add_custom_target(format_code_${PROJECT_NAME}
            COMMAND ${FIND} ${PROJECT_SOURCE_DIR} -name '*.h' -or -name '*.hpp' -or -name '*.cpp' | xargs ${CLANG_FORMAT} -i -style=file $1
            WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
            COMMENT "Reformatting files for ${PROJECT_NAME}")
endmacro()