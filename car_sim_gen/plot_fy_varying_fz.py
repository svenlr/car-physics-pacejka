import sys

import casadi
import numpy as np
import matplotlib.pyplot as plt

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

    plt.rcParams.update({'font.size': 18})

    Fz_vals = np.linspace(2, 10, 5)
    for Fz_val in Fz_vals:
        limit = 0.5
        vsy_vals = np.linspace(-limit, limit, num=200)
        vsy_vals_lin = np.linspace(-0.09, 0.09, num=200)
        Fy_vals = []
        Fx_vals = []
        p_val = [Fz_val] * 4 + p_val[4:]
        for vsy_val in vsy_vals:
            Fx_val, Fy_val = calc_forces(1, 1, vsy_val, p_val)
            Fy_vals.append(Fy_val)
            Fx_vals.append(Fx_val)
        plt.plot(np.arctan2(vsy_vals, 1), -np.array(Fy_vals))

    # plt.plot(vsy_vals, Fx_vals, label=r"$F_x$")
    # plt.plot(vsy_vals, np.clip(-18 * vsy_vals, np.min(Fy_vals), np.max(Fy_vals)), label=r"$F_y^{lin}$")
    # plt.plot(vsy_vals_lin, np.clip(-20 * vsy_vals_lin, np.min(Fy_vals), np.max(Fy_vals)), label=r"$F_y^{lin}$")
    plt.gca().set_xlabel(r"$\alpha$ [m/s]")
    plt.gca().set_ylabel(r"$F_y$ [N]")
    # plt.gca().set_title(r"$v_x=1.1$ $v_r=1$")
    plt.legend()

    plt.savefig("/home/sven/ma-mpc/tex/images/fy_varying_fz.pdf", bbox_inches="tight")
    plt.show()
