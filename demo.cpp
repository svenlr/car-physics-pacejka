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

    sim.u.dc = 0.1;
    // sim for 1 second
    step_sim(sim, x_traj, x_traj_approx, approx_state, 1000);
    // steer both front angles to the left
    sim.u = {0.2, 0.1, 0.1, 0, 0};
    // sim for 1 second
    step_sim(sim, x_traj, x_traj_approx, approx_state, 1000);
    // steer both front angles to the right while accelerating
    sim.u = {1.0, -0.1, -0.1, 0, 0};
    // sim for 1 second
    step_sim(sim, x_traj, x_traj_approx, approx_state, 1000);


    // plot the trajectory during the above simulation
    std::vector<double> s;
    std::vector<double> r;
    std::vector<double> omega;
    std::vector<double> v_x;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> xa;
    std::vector<double> ya;
    int step = 100;
    for (int i = 0; i < x_traj.size(); i += step) {
        const auto &state = x_traj[i];
        x.push_back(state.X);
        y.push_back(state.Y);
        s.push_back(i);
        r.push_back(state.r);
        omega.push_back(state.omega / 100);
        v_x.push_back(state.v_x);
    }
    for (int i = 0; i < x_traj_approx.size(); i += step) {
        const auto &state_approx = x_traj_approx[i];
        xa.push_back(state_approx.X);
        ya.push_back(state_approx.Y);
    }
//    plt::plot(x, y);
//    plt::plot(xa, ya, "x");
//    plt::axis("equal");
//    plt::show();

    plt::named_plot("r", s, r);
    plt::named_plot("omega", s, omega);
    plt::named_plot("v_x", s, v_x);
    plt::legend();
    plt::show();
}

void
step_sim(car_sim::CarSimulation &sim, std::vector<car_sim::state> &x_traj, std::vector<car_sim::state> &x_traj_approx,
         car_sim::state &approx_state, int n) {
    for (int i = 0; i < n; ++i) {
        sim.step();
        x_traj.push_back(sim.x);

        approx_state.v_x += 0.5 * sim.lin_acc_x * sim.get_time_step_size();
        approx_state.X += approx_state.v_x * sim.get_time_step_size();
        approx_state.v_x += 0.5 * sim.lin_acc_x * sim.get_time_step_size();

        approx_state.v_y += 0.5 * sim.lin_acc_y * sim.get_time_step_size();
        approx_state.Y += approx_state.v_y * sim.get_time_step_size();
        approx_state.v_y += 0.5 * sim.lin_acc_y * sim.get_time_step_size();

        approx_state.r += 0.5 * sim.ang_acc_phi * sim.get_time_step_size();
        approx_state.phi += approx_state.r * sim.get_time_step_size();
        approx_state.r += 0.5 * sim.ang_acc_phi * sim.get_time_step_size();
        x_traj_approx.push_back(approx_state);
    }
}
