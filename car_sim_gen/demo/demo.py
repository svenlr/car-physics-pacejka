import casadi
import matplotlib.pyplot as plt
import numpy as np

from car_sim_gen.car_model import calc_wheel_physics
from car_sim_gen.generate_car_sim import generate_car_sim
from car_sim_gen.constants import CarConstants


def step(sim_solver, u):
    sim_solver.set("x", sim_solver.get("x"))
    sim_solver.set("u", u)
    sim_solver.solve()


def main():
    sim_solver = generate_car_sim("/tmp/generated", time_step=0.001, num_stages=1, num_steps=1)
    sim_solver.set("u", np.array([1.0, 0, 0, 0, 0]))
    model = sim_solver.sim_struct.model
    c = CarConstants()
    Fx_i, Fx_w, Fy_i, car_torque_i = calc_wheel_physics(model, 0, c)
    wheel0_forces_func = casadi.Function("wheel0_forces_func", [model.x, model.u, model.p], [Fx_i, Fy_i, car_torque_i])
    x_traj = []
    for i in range(1000):
        step(sim_solver, np.array([0.1, 0, 0, 0, 0]))
        x_traj.append(sim_solver.get("x"))

    # 0.05 => R=5.1
    # 0.1 => R=2.55
    # 0.15 => R=1.75
    # 0.2 => R=1.32

    delta = 0.1
    j = 0.5
    for i in range(3000):
        step(sim_solver, np.array([j, delta, delta, 0, 0]))
        x_traj.append(sim_solver.get("x"))
        Fx_val, Fy_val, car_torque_val = wheel0_forces_func(sim_solver.get("x"), np.array([j, delta, delta, 0, 0]),
                           sim_solver.sim_struct.parameter_values)
        print(Fx_val, Fy_val, car_torque_val)
    for i in range(1000):
        step(sim_solver, np.array([0.6, 0.2, 0.2, 0, 0]))
        x_traj.append(sim_solver.get("x"))
    for i in range(1000):
        step(sim_solver, np.array([0.8, 0.0, 0.0, 0, 0]))
        x_traj.append(sim_solver.get("x"))
    for i in range(2000):
        step(sim_solver, np.array([0.5, -0.3, -0.3, 0, 0]))
        x_traj.append(sim_solver.get("x"))

    print("final state", sim_solver.get("x"))
    x_traj = np.array(x_traj)
    plt.plot(x_traj[:, 0], x_traj[:, 1])
    x_traj = x_traj[::max(int(x_traj.shape[0] / 50), 1), :]
    for x in x_traj:
        draw_car(x[0], x[1], x[2])
    plt.axis('equal')
    plt.gca().set_xlabel("x [m]")
    plt.gca().set_ylabel("y [m]")
    plt.savefig('../../doc/demo.png')
    plt.show()


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