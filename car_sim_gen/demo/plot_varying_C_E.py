import sys

import casadi
import numpy as np
import matplotlib.pyplot as plt

from car_sim_gen.car_model import calc_wheel_centric_velocities, create_car_model, calc_sigma_xy, \
    calc_wheel_centric_forces
from car_sim_gen.constants import WheelConstants
import matplotlib.patches as patches

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
    mu_x_val = cw0.mu_x0
    mu_y_val = cw0.mu_y0
    p_val = [cw0.Fz0] * c.n_wheels + [mu_x_val] * c.n_wheels + [mu_y_val] * c.n_wheels

    vx, vy = calc_wheel_centric_velocities(x, u, c, 0)
    sigma_x, sigma_y = calc_sigma_xy(vr, v_wx, v_wy)

    font_size=18
    plt.rcParams.update({'font.size': font_size})

    # for arrows
    style = "Simple, tail_width=0.2, head_width=5, head_length=8"
    kw = dict(arrowstyle=style, color="k")

    fig, plots = plt.subplots(1, 2,figsize=(12,5), sharey=True)
    # f, axs = plt.subplots(2,2,figsize=(15,15))

    D = 1
    E_vals = [-10, -2, 0, 0.7]

    for E_val in E_vals:
        limit = 0.5
        velocity = 1.0
        vsy_vals = np.linspace(-0, limit, num=200)
        Fy_vals = []
        Fx_vals = []
        cw0.Cy = 1.0
        cw0.Dy0 = 1
        cw0.Ey = E_val
        Fx, Fy = calc_wheel_centric_forces(sigma_x, sigma_y, p, cw0, 0)
        calc_slip = casadi.Function("calc_slip", [x, u], [vx, vy], dict())
        calc_forces = casadi.Function("calc_forces", [vr, v_wx, v_wy, p], [Fx, Fy], dict())
        for vsy_val in vsy_vals:
            Fx_val, Fy_val = calc_forces(1, velocity, vsy_val, p_val)
            Fy_vals.append(Fy_val)
            Fx_vals.append(Fx_val)
        plots[0].plot(np.arctan2(vsy_vals, velocity), -np.array(Fy_vals))

        a3 = patches.FancyArrowPatch((0.02, 0.385 * 2), (0.14, 0.28 * 2),
                                     connectionstyle="arc3,rad=-.1", **kw)
        plots[0].add_patch(a3)
        plots[0].text(0.02, 0.39 * 2, 'E', size=font_size, color='black')


    plots[0].plot([], [], label=r"$C=1.0$")
    plots[0].legend(handlelength=0, handletextpad=0)

    plots[0].set_xlabel(r"$x$")
    plots[0].set_ylabel(r"$F(x) / D$")


    cw0 = c.wheel_constants[1]
    D = 6
    E_vals = [-10, -2, 0, 0.7]

    for E_val in E_vals:
        limit = 0.5
        velocity = 1.0
        vsy_vals = np.linspace(-0, limit, num=200)
        Fy_vals = []
        Fx_vals = []
        cw0.Dy0 = 1
        cw0.Ey = E_val
        cw0.Cy = 1.5
        Fx, Fy = calc_wheel_centric_forces(sigma_x, sigma_y, p, cw0, 0)
        calc_slip = casadi.Function("calc_slip", [x, u], [vx, vy], dict())
        calc_forces = casadi.Function("calc_forces", [vr, v_wx, v_wy, p], [Fx, Fy], dict())
        for vsy_val in vsy_vals:
            Fx_val, Fy_val = calc_forces(1, velocity, vsy_val, p_val)
            Fy_vals.append(Fy_val)
            Fx_vals.append(Fx_val)
        plots[1].plot(np.arctan2(vsy_vals, velocity), -np.array(Fy_vals))

        a3 = patches.FancyArrowPatch((0.0, 0.37 * 2 + 0.2), (0.12, 0.32 * 2 + 0.1),
                                     connectionstyle="arc3,rad=-.1", **kw)
        plots[1].add_patch(a3)
        plots[1].text(0.00, 0.375 * 2 + 0.2, 'E', size=font_size, color='black')

    # plt.plot(vsy_vals, Fx_vals, label=r"$F_x$")
    # plt.plot(vsy_vals, np.clip(-18 * vsy_vals, np.min(Fy_vals), np.max(Fy_vals)), label=r"$F_y^{lin}$")
    # plt.plot(vsy_vals_lin, np.clip(-20 * vsy_vals_lin, np.min(Fy_vals), np.max(Fy_vals)), label=r"$F_y^{lin}$")
    plt.gca().set_xlabel("$x$")
    plots[1].plot([], [], label=r"$C=1.5$")
    plt.legend(handlelength=0, handletextpad=0)
    # plt.gca().set_ylabel(r"$F_y$ [N]")
    # plt.gca().set_title(r"$v_x=1.1$ $v_r=1$")

    plt.show()
