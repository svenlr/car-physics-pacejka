import sys

import casadi
import numpy as np
import matplotlib.pyplot as plt

from car_sim_gen.car_model import calc_wheel_centric_velocities, create_car_model, calc_sigma_xy, \
    calc_wheel_centric_forces
from car_sim_gen.constants import WheelConstants

if __name__ == '__main__':
    model, c, q = create_car_model()
    vr = casadi.MX.sym("vr")
    v_wx = casadi.MX.sym("v_wx")
    v_wy = casadi.MX.sym("v_wy")
    v_x = casadi.MX.sym("v_x")
    v_y = casadi.MX.sym("v_y")
    r = casadi.MX.sym("r")
    delta = casadi.MX.sym("delta0")
    mu_x = casadi.MX.sym("mu_x", c.n_wheels, 1)
    mu_y = casadi.MX.sym("mu_y", c.n_wheels, 1)
    Fz = casadi.MX.sym("Fz", c.n_wheels, 1)
    x = casadi.vertcat(v_x, v_y, r)
    u = casadi.vertcat(delta)
    p = casadi.vertcat(Fz, mu_x, mu_y)
    cw0: WheelConstants = c.wheel_constants[0]
    mu_x_val = 0.5
    mu_y_val = 0.4
    p_val = [cw0.Fz0] * c.n_wheels + [mu_x_val] * c.n_wheels + [mu_y_val] * c.n_wheels

    vx, vy = calc_wheel_centric_velocities(x, u, c, 0)
    sigma_x, sigma_y = calc_sigma_xy(vr, v_wx, v_wy)
    Fx, Fy = calc_wheel_centric_forces(sigma_x, sigma_y, p, c.wheel_constants[0], 0)
    calc_slip = casadi.Function("calc_slip", [x, u], [vx, vy], dict())
    calc_forces = casadi.Function("calc_forces", [vr, v_wx, v_wy, p], [Fx, Fy], dict())

    print(calc_slip([0, 0, -1], [0]))
    print(calc_forces(0, 0, 0, [3] * 4 + [1] * 8))

    fig, plots = plt.subplots(2, 2)

    vsy_vals = np.linspace(-1.0, 1.0)
    Fy_vals = []
    Fx_vals = []
    for vsy_val in vsy_vals:
        Fx_val, Fy_val = calc_forces(1, 1.1, vsy_val, p_val)
        Fy_vals.append(np.squeeze(Fy_val))
        Fx_vals.append(np.squeeze(Fx_val))
    plots[0,0].plot(vsy_vals, Fy_vals, label="Fy")
    plots[0,0].plot(vsy_vals, Fx_vals, label="Fx")
    plots[0,0].set_xlabel("Vsy [m/s]")
    plots[0,0].set_ylabel("F [N]")
    plots[0,0].set_title('Vx=1.1 Vr=1')
    plots[0,0].legend()

    vr_vals = np.linspace(0.5, 1.5, num=100)
    vsy_val = 0.05
    Fy_vals = []
    Fx_vals = []
    for vr_val in vr_vals:
        Fx_val, Fy_val = calc_forces(vr_val, 1.0, vsy_val, p_val)
        Fy_vals.append(np.squeeze(Fy_val))
        Fx_vals.append(np.squeeze(Fx_val))
    plots[1,0].plot(vr_vals, Fy_vals, label="Fy [N]")
    plots[1,0].plot(vr_vals, Fx_vals, label="Fx [N]")
    plots[1,0].set_xlabel("Vr [m/s]")
    plots[1,0].set_ylabel("F [N]")
    plots[1,0].set_title('Vx=1.0 Vsy=0.05')
    plots[1,0].legend()

    Fz_vals = np.linspace(1, 10, num=5)
    vr_vals = np.linspace(0.5, 1.5, num=100)
    vsy_val = 0.0
    for Fz_val in Fz_vals:
        Fx_vals = []
        for vr_val in vr_vals:
            Fx_val, Fy_val = calc_forces(vr_val, 1.0, vsy_val,
                                         [Fz_val] * c.n_wheels + [mu_x_val] * c.n_wheels + [mu_y_val] * c.n_wheels)
            Fx_vals.append(np.squeeze(Fx_val))
        plots[0,1].plot(vr_vals, Fx_vals)
    plots[0,1].set_xlabel("Vr [m/s]")
    plots[0,1].set_ylabel("Fx [N]")
    plots[0,1].set_title('Vx=1.0 Vsy=0.0 (Varying Fz)')

    Fz_vals = np.linspace(0, 10, num=100)
    vsy_val = -0.1
    Fy_vals = []
    for Fz_val in Fz_vals:
        Fx_val, Fy_val = calc_forces(1.0, 1.0, vsy_val,
                                     [Fz_val] * c.n_wheels + [mu_x_val] * c.n_wheels + [mu_y_val] * c.n_wheels)
        Fy_vals.append(np.squeeze(Fy_val))
    plt.plot(Fz_vals, Fy_vals)
    plots[1,1].set_xlabel('Fz [N]')
    plots[1,1].set_ylabel('Fy [N]')
    plots[1,1].set_title('Vx=Vr=1.0, Vsx=0 Vsy=-0.1')
    plt.tight_layout(2.0)
    plt.show()
