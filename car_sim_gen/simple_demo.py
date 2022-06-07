import os

import casadi
import matplotlib.pyplot as plt
import numpy as np
from acados_template import AcadosSim, AcadosSimSolver

from car_sim_gen.simple_car_model import create_simple_bicycle_car_model, calc_forces, CarModelParameters, \
    CarModelPhysicalQuantities
from generate_car_sim import generate_car_sim


def step(sim_solver, u):
    sim_solver.set("x", sim_solver.get("x"))
    sim_solver.set("u", u)
    sim_solver.solve()


def main():
    sim_solver = generate_simple_car_sim("/tmp/generated_simple", time_step=0.001, num_stages=1, num_steps=1)
    sim_solver.set("u", np.array([1.0, 0, 0]))
    p = CarModelParameters()
    q = CarModelPhysicalQuantities()
    model = sim_solver.sim_struct.model
    F_y_f, F_y_r, Fxf, Fxr = calc_forces(sim_solver.sim_struct.model, p, q)
    forces_func = casadi.Function("calc_forces", [model.x, model.u], [F_y_f, F_y_r])
    x_traj = []
    for i in range(1000):
        step(sim_solver, np.array([0.1, 0, 0]))
        x_traj.append(sim_solver.get("x"))
    for i in range(1000):
        step(sim_solver, np.array([0.5, 0, 0]))
        x_traj.append(sim_solver.get("x"))

    delta = 0.1
    for j in np.linspace(0.5,  0.7, num=10):
        for i in range(5000):
            step(sim_solver, np.array([j, delta, 0]))
            x_traj.append(sim_solver.get("x"))
        print(forces_func(sim_solver.get("x"), np.array([j, delta, 0])))
    # for i in range(1000):
    #     step(sim_solver, np.array([1.0, 0, 0]))
    #     x_traj.append(sim_solver.get("x"))
    # for i in range(5000):
    #     step(sim_solver, np.array([j, delta, 0]))
    #     x_traj.append(sim_solver.get("x"))

    print("final state", sim_solver.get("x"))
    x_traj = np.array(x_traj)
    plt.plot(x_traj[:, 0], x_traj[:, 1])
    x_traj = x_traj[::max(int(x_traj.shape[0] / 50), 1), :]
    for x in x_traj:
        draw_car(x[0], x[1], x[2])
    plt.axis('equal')
    plt.gca().set_xlabel("x [m]")
    plt.gca().set_ylabel("y [m]")
    plt.savefig('../doc/demo.png')
    plt.show()


def generate_simple_car_sim(code_export_dir, time_step=0.001, num_stages=1, num_steps=1):
    model, c, q = create_simple_bicycle_car_model()
    sim = AcadosSim()
    sim.model = model

    sim.solver_options.T = time_step
    sim.solver_options.integrator_type = "ERK"
    sim.solver_options.num_stages = num_stages
    sim.solver_options.num_steps = num_steps
    sim.code_export_directory = code_export_dir

    sim.parameter_values = np.array([])

    return AcadosSimSolver(sim, json_file=os.path.join(sim.code_export_directory, "acados_sim.json"))


def draw_car(x, y, phi):
    w = 0.35
    h = 0.18
    pos = np.array([x, y])
    v = np.array([np.cos(phi), np.sin(phi)])
    o = np.array([-np.sin(phi), np.cos(phi)])
    positions = np.array([
        pos + v * w / 2 + o * h / 2,
        pos + v * w / 2 - o * h / 2,
        pos - v * w / 2 - o * h / 2,
        pos - v * w / 2 + o * h / 2,
        pos + v * w / 2 + o * h / 2,
    ])
    plt.plot(positions[:, 0], positions[:, 1], "-", color="black")
    # vf = np.array([np.cos(phi + delta_f), np.sin(phi + delta_f)])
    # vr = np.array([np.cos(phi + delta_r), np.sin(phi + delta_r)])
    # f_steering = np.array([
    #     pos + v * 0.13 + vf * 0.06,
    #     pos + v * 0.13 - vf * 0.06,
    # ])
    # r_steering = np.array([
    #     pos - v * 0.13 + vr * 0.06,
    #     pos - v * 0.13 - vr * 0.06,
    # ])
    # plt.plot(f_steering[:, 0], f_steering[:, 1], "-", color="black", lw=3)
    # plt.plot(r_steering[:, 0], r_steering[:, 1], "-", color="black", lw=3)


if __name__ == '__main__':
    main()