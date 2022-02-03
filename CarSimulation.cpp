//
// Created by sven on 1/9/22.
//

#include "CarSimulation.h"

#include "acados_sim_solver_car.h"

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
    sim_in_set(sim->acados_sim_config, sim->acados_sim_dims,
               sim->acados_sim_in, "x", &x);
    sim_in_set(sim->acados_sim_config, sim->acados_sim_dims,
               sim->acados_sim_in, "u", &u);
    int status = car_acados_sim_solve(sim);
    sim_out_get(sim->acados_sim_config, sim->acados_sim_dims,
                sim->acados_sim_out, "x", &x);
    return status;
}

double CarSimulation::get_time_step_size() const {
    return TIME_STEP;
}

int CarSimulation::num_x() const {
    return CAR_NX;
}

int CarSimulation::num_u() const {
    return CAR_NU;
}
