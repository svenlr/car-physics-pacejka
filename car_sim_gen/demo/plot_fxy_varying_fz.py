import sys

import casadi
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from car_model import calc_wheel_centric_velocities, create_car_model, calc_sigma_xy, calc_wheel_centric_forces
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

    # for arrows
    style = "Simple, tail_width=0.2, head_width=5, head_length=8"
    kw = dict(arrowstyle=style, color="k")

    font_size=18
    plt.rcParams.update({'font.size': font_size})

    fig, plots = plt.subplots(1, 2,figsize=(12,5), sharey=True)

    Fz_vals = np.linspace(2, 10, 5)
    for Fz_val in Fz_vals:
        vsx_vals = np.linspace(-0.0, 0.2, num=200)
        Fy_vals = []
        Fx_vals = []
        p_val = [Fz_val] * 4 + p_val[4:]
        for vsx_val in vsx_vals:
            Fx_val, Fy_val = calc_forces(1, 1 + vsx_val, 0, p_val)
            Fx_vals.append(Fx_val)

        plots[0].plot(vsx_vals, -np.array(Fx_vals))

    plots[0].plot([],[], 'k', label=r"$F_x$")

    a3 = patches.FancyArrowPatch((0.07, 0.5), (0.13, 5.0),
                                 connectionstyle="arc3,rad=-.1", **kw)
    plots[0].add_patch(a3)
    plots[0].text(0.074, 0.3, r"$F_z$", size=font_size, color='black')

    plots[0].set_xlabel(r"$\kappa$ [%]")
    plots[0].set_ylabel(r"$F$ [N]")

    plots[0].legend(handlelength=0, handletextpad=0)

    Fz_vals = np.linspace(2, 10, 5)
    for Fz_val in Fz_vals:
        limit = 0.5
        vsy_vals = np.linspace(-0, limit, num=200)
        Fy_vals = []
        Fx_vals = []
        p_val = [Fz_val] * 4 + p_val[4:]
        for vsy_val in vsy_vals:
            Fx_val, Fy_val = calc_forces(1, 1, vsy_val, p_val)
            Fy_vals.append(Fy_val)
            Fx_vals.append(Fx_val)
        plots[1].plot(np.arctan2(vsy_vals, 1), -np.array(Fy_vals))


    a3 = patches.FancyArrowPatch((0.2, 0.2), (0.3, 4.5),
                                 connectionstyle="arc3,rad=-.1", **kw)
    plots[1].add_patch(a3)
    plots[1].text(0.21, -0.0, r"$F_z$", size=font_size, color='black')

    plots[1].plot([],[], 'k', label=r"$F_y$")

    # plt.plot(vsy_vals, Fx_vals, label=r"$F_x$")
    # plt.plot(vsy_vals, np.clip(-18 * vsy_vals, np.min(Fy_vals), np.max(Fy_vals)), label=r"$F_y^{lin}$")
    # plt.plot(vsy_vals_lin, np.clip(-20 * vsy_vals_lin, np.min(Fy_vals), np.max(Fy_vals)), label=r"$F_y^{lin}$")
    plt.gca().set_xlabel(r"$\alpha$ [rad]")
    # plt.gca().set_ylabel(r"$F_x$ [N]")
    # plt.gca().set_title(r"$v_x=1.1$ $v_r=1$")
    plt.legend(handlelength=0, handletextpad=0)

    plt.savefig("/home/sven/ma-mpc/tex/images/fxy_varying_fz.pdf", bbox_inches="tight")
    plt.show()
