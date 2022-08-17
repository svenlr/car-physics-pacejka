//
// Created by sven on 1/9/22.
//

#include "CarSimulation.h"

#include "acados_sim_solver_car.h"

#include <cmath>

using namespace car_sim;

CarSimulation::CarSimulation() {
    sim = car_acados_sim_solver_create_capsule();
    car_acados_sim_create(sim);
}

CarSimulation::~CarSimulation() {
    car_acados_sim_free(sim);
    car_acados_sim_solver_free_capsule(sim);
}

int CarSimulation::step() {
    state x0 = x;
    car_acados_sim_update_params(sim, (double *) &p, int(CAR_NP));
    sim_in_set(sim->acados_sim_config, sim->acados_sim_dims,
               sim->acados_sim_in, "x", &x);
    sim_in_set(sim->acados_sim_config, sim->acados_sim_dims,
               sim->acados_sim_in, "u", &u);
    int status = car_acados_sim_solve(sim);
    sim_out_get(sim->acados_sim_config, sim->acados_sim_dims,
                sim->acados_sim_out, "x", &x);
    calc_accelerations_on_car(x0);
    return status;
}

void CarSimulation::calc_accelerations_on_car(const state &x0) {
    double vx0_world = x0.v_x * std::cos(x0.phi) - x0.v_y * std::sin(x0.phi);
    double vy0_world = x0.v_x * std::sin(x0.phi) + x0.v_y * std::cos(x0.phi);
    double vx_world = x.v_x * std::cos(x.phi) - x.v_y * std::sin(x.phi);
    double vy_world = x.v_x * std::sin(x.phi) + x.v_y * std::cos(x.phi);
    lin_acc_x = (vx_world - vx0_world) / get_time_step_size();
    lin_acc_y = (vy_world - vy0_world) / get_time_step_size();
    ang_acc_phi = (x.r - x0.r) / get_time_step_size();
}

double CarSimulation::get_time_step_size() const {
    return TIME_STEP;
}
