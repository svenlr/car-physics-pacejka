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

        /** no copy allowed because of the C pointer */
        CarSimulation(const CarSimulation &other) = delete;

        /** no copy allowed because of the C pointer */
        CarSimulation &operator=(const CarSimulation &other) = delete;

    public:
        state x;
        control u;
        params p;
        double avg_lin_acc_x = 0;
        double avg_lin_acc_y = 0;
        double avg_ang_acc_phi = 0;

    private:
        sim_solver_capsule *sim = nullptr;

    private:
        void calc_accelerations_on_car(const state &x0);
    };

}