import sys

import casadi
import numpy as np
import matplotlib.pyplot as plt

from car_sim_gen.car_model import calc_wheel_centric_velocities, create_car_model, calc_sigma_xy, \
    calc_wheel_centric_forces, calc_wheel_physics
from car_sim_gen.constants import WheelConstants

from acados_template.acados_ocp_formulation_helper import get_symbol_idx


def main():
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

    Fx_i, Fx_w, Fy_i, car_torque_i = calc_wheel_physics(model, 0, c)
    model_func = casadi.Function("model_func", [model.x, model.u, model.p], [Fx_i, Fy_i, car_torque_i])

    vx_vals = np.linspace(0.0, 3.0)
    torque_vals = []
    Fx_vals = []

    x_val = np.zeros(shape=(model.x.size()[0],))
    u_val = np.zeros(shape=(model.u.size()[0],))
    for vx_val in vx_vals:
        x_val[get_symbol_idx(model.x, "v_x")] = vx_val
        # x_val[get_symbol_idx(model.x, "r")] = vx_val
        omega = x_val[get_symbol_idx(model.x, "omega")] = vx_val / c.wheel_constants[0].radius
        dc = (c.Tm_emv * omega + c.Tm_drag * omega * omega * np.sign(omega)) / c.Tm_p
        u_val[get_symbol_idx(model.x, "dc")] = dc
        u_val[get_symbol_idx(model.x, "delta0")] = 0.1
        u_val[get_symbol_idx(model.x, "delta1")] = 0.1
        fx, fy, t = model_func(x_val, u_val, p_val)
        Fx_vals.append(np.squeeze(fx))
        torque_vals.append(np.squeeze(t))
    plt.plot(vx_vals, torque_vals, label="torque")
    plt.plot(vx_vals, Fx_vals, label="Fx")
    plt.gca().set_xlabel("vx [m/s]")
    plt.gca().set_ylabel("Fx [N] / Torque [Nm]")
    plt.gca().set_title('Fx and Torque at a single wheel with increasing vx')
    plt.legend()

    plt.show()


if __name__ == '__main__':
    main()
