cmake_minimum_required(VERSION 3.20)
project(car_physics_pacejka)

## Compile as C++17
add_compile_options(-std=c++17)

include(cmake/add_generated_library.cmake)
include(cmake/download_matplotlib.cmake)

file(GLOB_RECURSE PY_SIM_GEN_FILES CONFIGURE_DEPENDS car_sim_gen/*)

if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/generated)
    message("-- Creating directory ${CMAKE_CURRENT_SOURCE_DIR}/generated")
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/generated)
endif()

find_package(acados REQUIRED)

cmake_path(GET acados_DIR PARENT_PATH acados_INSTALL_DIR)

set(acados_LIBRARY_DIRS ${acados_INSTALL_DIR}/lib)
set(acados_INCLUDE_DIRS ${acados_INSTALL_DIR}/include)

message("${acados_INCLUDE_DIRS}")

add_generated_library(
        TARGET acados_sim_solver_car
        COMMAND car_sim_gen/generate_car_sim.py ${acados_INSTALL_DIR}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/generated/libacados_sim_solver_car.so
        DEPENDS ${PY_SIM_GEN_FILES}
        INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/generated
        )

# get_cmake_property(_variableNames VARIABLES)
# list (SORT _variableNames)
# foreach (_variableName ${_variableNames})
#     message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()

add_library(car_physics_pacejka SHARED
        CarSimulation.cpp
        )

include_directories(${acados_INCLUDE_DIRS})

target_include_directories(car_physics_pacejka SYSTEM PUBLIC
        ${acados_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/generated
        )

target_link_directories(car_physics_pacejka PUBLIC ${acados_LIBRARY_DIRS})
target_link_libraries(car_physics_pacejka
        -lacados -lhpipm -lblasfeo -lm
        acados_sim_solver_car
        )


if (Python3_FOUND)
    add_executable(demo CarSimulation.cpp demo.cpp)
    target_include_directories(demo SYSTEM PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/generated ${matplotlib_INCLUDE_DIRS}
            ${acados_INCLUDE_DIRS})
    target_link_libraries(demo ${matplotlib_LIBRARIES} ${acados_LIBRARIES} acados_sim_solver_car)
endif()
