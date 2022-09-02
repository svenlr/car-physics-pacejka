function(add_generated_library)
    set(oneValueArgs TARGET OUTPUT WORKING_DIRECTORY)
    set(multiValueArgs COMMAND INCLUDE_DIRS DEPENDS)
    cmake_parse_arguments(GEN_LIB "" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    add_custom_command(
            COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH="$ENV{PYTHONPATH}:${CMAKE_CURRENT_SOURCE_DIR}" ${GEN_LIB_COMMAND}
            OUTPUT "${GEN_LIB_OUTPUT}"
            WORKING_DIRECTORY "${GEN_LIB_WORKING_DIRECTORY}"
            DEPENDS ${GEN_LIB_DEPENDS}
            )
    add_library(${GEN_LIB_TARGET} SHARED IMPORTED GLOBAL)
    set_target_properties(${GEN_LIB_TARGET} PROPERTIES
        IMPORTED_LOCATION "${GEN_LIB_OUTPUT}"
        INTERFACE_INCLUDE_DIRECTORIES "${GEN_LIB_INCLUDE_DIRS}"
        )
    add_custom_target(${GEN_LIB_TARGET}_generate DEPENDS ${GEN_LIB_OUTPUT})
    add_dependencies(${GEN_LIB_TARGET} ${GEN_LIB_TARGET}_generate)
endfunction()
