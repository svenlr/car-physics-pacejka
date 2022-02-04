//
// Created by sven on 2/4/22.
//

#include "CarSimulation.h"
#include "matplotlibcpp.h"

void
step_sim(car_sim::CarSimulation &sim, std::vector<car_sim::state> &x_traj, std::vector<car_sim::state> &x_traj_approx,
         car_sim::state &approx_state, int n);

int main(int argc, char **argv) {
    namespace plt = matplotlibcpp;

    car_sim::CarSimulation sim;
    std::vector<car_sim::state> x_traj;
    std::vector<car_sim::state> x_traj_approx;

    car_sim::state approx_state;

    sim.u.dc = 0.5;
    step_sim(sim, x_traj, x_traj_approx, approx_state, 1000);
    sim.u = {0.8, 0.2, 0.2, 0, 0};
    step_sim(sim, x_traj, x_traj_approx, approx_state, 1000);
    sim.u = {1.0, 0.0, 0.0, 0, 0};
    step_sim(sim, x_traj, x_traj_approx, approx_state, 1000);
    sim.u = {1.0, -0.3, -0.3, 0, 0};
    step_sim(sim, x_traj, x_traj_approx, approx_state, 2000);

    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> xa;
    std::vector<double> ya;
    int step = 100;
    for (int i = 0; i < x_traj.size(); i += step) {
        const auto &state = x_traj[i];
        x.push_back(state.X);
        y.push_back(state.Y);
    }
    for (int i = 0; i < x_traj_approx.size(); i += step) {
        const auto &state_approx = x_traj_approx[i];
        xa.push_back(state_approx.X);
        ya.push_back(state_approx.Y);
    }
    plt::plot(x, y);
    plt::plot(xa, ya, "x");
    plt::axis("equal");
    plt::show();
}

void
step_sim(car_sim::CarSimulation &sim, std::vector<car_sim::state> &x_traj, std::vector<car_sim::state> &x_traj_approx,
         car_sim::state &approx_state, int n) {
    for (int i = 0; i < n; ++i) {
        sim.step();
        x_traj.push_back(sim.x);

        approx_state.v_x += 0.5 * sim.avg_lin_acc_x * sim.get_time_step_size();
        approx_state.X += approx_state.v_x * sim.get_time_step_size();
        approx_state.v_x += 0.5 * sim.avg_lin_acc_x * sim.get_time_step_size();

        approx_state.v_y += 0.5 * sim.avg_lin_acc_y * sim.get_time_step_size();
        approx_state.Y += approx_state.v_y * sim.get_time_step_size();
        approx_state.v_y += 0.5 * sim.avg_lin_acc_y * sim.get_time_step_size();

        approx_state.r += 0.5 * sim.avg_ang_acc_phi * sim.get_time_step_size();
        approx_state.phi += approx_state.r * sim.get_time_step_size();
        approx_state.r += 0.5 * sim.avg_ang_acc_phi * sim.get_time_step_size();
        x_traj_approx.push_back(approx_state);
    }
}
