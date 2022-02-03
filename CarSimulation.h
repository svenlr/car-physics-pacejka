//
// Created by sven on 1/9/22.
//
#pragma once

#include <vector>
#include <car_model_types.h>

struct sim_solver_capsule;

namespace car_sim {

    class CarSimulation {
    public:
        CarSimulation();

        ~CarSimulation();

        int step();

        double get_time_step_size() const;

        int num_x() const;

        int num_u() const;

        /** no copy allowed because of the C pointer */
        CarSimulation(const CarSimulation &other) = delete;

        /** no copy allowed because of the C pointer */
        CarSimulation &operator=(const CarSimulation &other) = delete;

    public:
        state x;
        control u;

    private:
        sim_solver_capsule *sim = nullptr;
    };

}