cmake_minimum_required(VERSION 3.5)
project(car_physics_pacejka)

## Compile as C++14
add_compile_options(-std=c++14)

if(NOT DEFINED ENV{ACADOS_SOURCE_DIR})
    message(ERROR "Either acados is not installed or ACADOS_SOURCE_DIR is unset.")
    return()
endif()

include(cmake/add_generated_library.cmake)

file(GLOB_RECURSE PY_SIM_GEN_FILES CONFIGURE_DEPENDS car_sim_gen/*)

add_generated_library(
        TARGET acados_sim_solver_car
        COMMAND python3 car_sim_gen/generate_car_sim.py
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/generated/libacados_sim_solver_car.so
        DEPENDS ${PY_SIM_GEN_FILES}
        INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/generated
        )

set(ACADOS_INCLUDE_DIRS
        $ENV{ACADOS_SOURCE_DIR}/include
        $ENV{ACADOS_SOURCE_DIR}/include/blasfeo/include
        $ENV{ACADOS_SOURCE_DIR}/include/hpipm/include
        )
set(ACADOS_LIBRARIES
        -L$ENV{ACADOS_SOURCE_DIR}/lib -lacados -lhpipm -lblasfeo -lm
        )

add_library(car_physics_pacejka SHARED
        CarSimulation.cpp
        )
target_include_directories(car_physics_pacejka SYSTEM PRIVATE
        ${ACADOS_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/generated
        )
target_link_libraries(car_physics_pacejka
        ${ACADOS_LIBRARIES}
        acados_sim_solver_car
        )
