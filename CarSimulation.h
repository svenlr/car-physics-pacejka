//
// Created by sven on 1/9/22.
//
#pragma once

#include <vector>
#include <car_model_types.h>

struct sim_solver_capsule;

namespace car_sim {

    /**
     * Wrapper class for the code generated with acados.
     */
    class CarSimulation {
    public:
        /**
         * initialized the RK4 solver
         */
        CarSimulation();

        /**
         * free the solver
         */
        ~CarSimulation();

        /**
         * Perform a simulation step based on the current values for x,u,p, then update lin_acc_x,lin_acc_y,ang_acc_phi
         * After performing a simulation step, the output (linear accelerations, angular acceleration)
         * can be read from lin_acc_x, lin_acc_y and ang_acc_phi.
         * @return acados status code
         */
        int step();

        /**
         * @return simulation step size
         */
        double get_time_step_size() const;

        /** no copy allowed because of the C pointer */
        CarSimulation(const CarSimulation &other) = delete;

        /** no copy allowed because of the C pointer */
        CarSimulation &operator=(const CarSimulation &other) = delete;

    public:
        state x;
        control u;
        params p;
        /** linear acceleration in x direction to be applied to the rigid body */
        double lin_acc_x = 0;
        /** linear acceleration in y direction to be applied to the rigid body */
        double lin_acc_y = 0;
        /** angular acceleration to be applied to the CoG of the rigid body */
        double ang_acc_phi = 0;

    private:
        sim_solver_capsule *sim = nullptr;

    private:
        void calc_accelerations_on_car(const state &x0);
    };

}