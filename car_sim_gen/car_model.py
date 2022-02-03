from acados_template import AcadosModel, AcadosOcpConstraints, AcadosOcp
from casadi import tanh, sin, cos, MX, types, interpolant, vertcat, Function, atan, sign, tan
import casadi
import numpy as np

from acados_template.acados_ocp_formulation_helper import get_symbol, auto_xdot

from constants import CarConstants, WheelConstants
from quantities import CarPhysicalQuantities, WheelPhysicalQuantities


def create_car_model(constants=None, model_name="car"):
    model = AcadosModel()
    model.name = model_name
    c = constants if constants is not None else CarConstants()
    q = CarPhysicalQuantities(c.n_wheels)

    # CasADi Model

    # states
    X = MX.sym("X")
    Y = MX.sym("Y")
    phi = MX.sym("phi")
    v_x = MX.sym("v_x")
    v_y = MX.sym("v_y")
    r = MX.sym("r")  # r = yaw rate, often called omega as well
    # transmissions are not modeled => the motor / differential dynamics should include any transmission dynamics
    omega = MX.sym("omega")  # base gear angular velocity
    d_omega_f = MX.sym("d_omega_f")  # offset of angular velocities at the front differential
    d_omega_r = MX.sym("d_omega_r")  # offset of angular velocities at the rear differential

    # controls
    dc = MX.sym("dc")
    # each steering angle is a separate control
    deltas = vertcat(*[MX.sym("delta{}".format(i)) for i in range(c.n_wheels)])

    # algebraic variables
    model.z = vertcat([])

    # parameters
    model.p = vertcat(
        MX.sym("Fz", c.n_wheels, 1),
        MX.sym("mu_x", c.n_wheels, 1),
        MX.sym("mu_y", c.n_wheels, 1),
    )

    model.u = vertcat(
        dc,
        deltas,
    )

    model.x = vertcat(
        X,
        Y,
        phi,
        v_x,
        v_y,
        r,
        omega,
        d_omega_f,
        d_omega_r,
    )

    q.wheel_quantities[0].v_r = (omega + d_omega_f / 2) * c.wheel_constants[0].radius
    q.wheel_quantities[1].v_r = (omega - d_omega_f / 2) * c.wheel_constants[1].radius
    q.wheel_quantities[2].v_r = (omega + d_omega_r / 2) * c.wheel_constants[2].radius
    q.wheel_quantities[3].v_r = (omega - d_omega_r / 2) * c.wheel_constants[3].radius

    Fx = None
    Fy = None
    car_torque = None
    tire_fb_torque = None
    tire_back_torques = [0.] * c.n_wheels
    for i in range(c.n_wheels):
        vr = q.wheel_quantities[i].v_r
        vx_w, vy_w = calc_wheel_centric_velocities(model.x, model.u, c, i)
        kappa, tan_alpha = calc_sigma_xy(vr, vx_w, vy_w)
        Fx_w, Fy_w = calc_wheel_centric_forces(kappa, tan_alpha, model.p, c.wheel_constants[i], i)
        Fx_i, Fy_i = wheel_force_to_car_force(model.u, i, Fx_w, Fy_w)
        car_torque_i = car_force_to_car_torque(c, i, Fx_i, Fy_i)
        car_torque = car_torque + car_torque_i if car_torque is not None else car_torque_i
        Fx = Fx + Fx_i if Fx is not None else Fx_i
        Fy = Fy + Fy_i if Fy is not None else Fy_i
        tire_back_torques[i] = -Fx_w * c.wheel_constants[i].radius
        tire_fb_torque = tire_fb_torque + tire_back_torques[i] if tire_fb_torque is not None else tire_back_torques[i]

    # motor torque
    Tm = calc_motor_torque(c, dc, omega)
    # total drive train inertia
    Iw0 = c.wheel_constants[0].I_w
    Im = np.sum([cw.I_w for cw in c.wheel_constants]) + c.Im

    # outer torque at front differential caused by a difference in tire friction
    diff_f_tire_fb_torque = tire_back_torques[0] - tire_back_torques[1]
    diff_r_tire_fb_torque = tire_back_torques[2] - tire_back_torques[3]

    # differential internal friction
    diff_f_damping_torque = -c.mu_diff_visc * d_omega_f * (casadi.fabs(Tm / 2) + 0.1)
    diff_r_damping_torque = -c.mu_diff_visc * d_omega_r * (casadi.fabs(Tm / 2) + 0.1)

    # left hand side for differential equations
    model.xdot = auto_xdot(model.x)
    # right hand side for differential equations
    model.f_expl_expr = vertcat(
        v_x * cos(phi) - v_y * sin(phi),  # = X_dot
        v_x * sin(phi) + v_y * cos(phi),  # = Y_dot
        r,  # = phi_dot
        (1 / c.m) * Fx + v_y * r,  # = v_x_dot
        (1 / c.m) * Fy - v_x * r,  # = v_y_dot
        (1 / c.I_z) * car_torque,  # r_dot
        (1 / Im) * (Tm + tire_fb_torque),
        (1 / (Iw0 * 2)) * (diff_f_damping_torque + diff_f_tire_fb_torque),
        (1 / (Iw0 * 2)) * (diff_r_damping_torque + diff_r_tire_fb_torque),
    )
    model.f_impl_expr = model.xdot - model.f_expl_expr

    return model, c, q


def calc_motor_torque(c, dc, omega):
    return c.Tm_p * dc - c.Tm_emv * omega - c.Tm_drag * omega * omega * sign(omega)


def calc_sigma_xy(vr, vx, vy):
    vsy = vy
    vsx = vx - vr
    vr_safe = casadi.fmax(casadi.fabs(vr), 1e-3)
    sigma_x = -vsx / vr_safe
    sigma_y = -vsy / vr_safe
    return sigma_x, sigma_y


def calc_wheel_centric_velocities(x: MX, u: MX, c: CarConstants, wheel_idx: int):
    delta = get_symbol(u, "delta{}".format(wheel_idx))
    cw = c.wheel_constants[wheel_idx]
    vx = get_symbol(x, "v_x")
    vy = get_symbol(x, "v_y")
    r = get_symbol(x, "r")
    cos_delta, sin_delta = cos(delta), sin(delta)
    # velocities of the wheel strut in the world frame
    vx_strut, vy_strut = (vx - cw.y * r), (vy + cw.x * r)
    # finally rotate from the strut coordinate system to the wheel coordinate system
    vx_wheel = vx_strut * cos_delta + vy_strut * sin_delta
    vy_wheel = vx_strut * -sin_delta + vy_strut * cos_delta
    return vx_wheel, vy_wheel


def calc_wheel_centric_forces(sigma_x: MX, sigma_y: MX, p: MX, cw: WheelConstants, wheel_idx: int):
    Fz = get_symbol(p, "Fz")[wheel_idx]
    mu_y = get_symbol(p, "mu_y")[wheel_idx]
    mu_x = get_symbol(p, "mu_x")[wheel_idx]

    C_Fa = cw.c1 * cw.c2 * cw.Fz0 * sin(cw.Cz * atan(Fz / (cw.c2 * cw.Fz0)))
    C_Fk = cw.c8 * Fz

    sigma = casadi.fmax(casadi.sqrt(sigma_x * sigma_x + sigma_y * sigma_y), 1e-3)

    sigma_x_eq = (C_Fk / cw.C_Fk0) * (cw.mu_x0 / mu_x) * (cw.Fz0 / Fz) * sigma
    kappa_eq = sigma_x_eq / casadi.fmax(1 - sigma_x_eq, 1e-3)  # abs(sigma_x) ?
    Fx0 = cw.Dx0 * sin(cw.Cx * atan(cw.Bx0 * kappa_eq - cw.Ex * (cw.Bx0 * kappa_eq - atan(cw.Bx0 * kappa_eq))))
    Fx = (sigma_x / sigma) * (mu_x / cw.mu_x0) * (Fz / cw.Fz0) * Fx0

    sigma_y_eq = (C_Fa / cw.C_Fa0) * (cw.mu_y0 / mu_y) * (cw.Fz0 / Fz) * sigma
    alpha_eq = atan(sigma_y_eq)
    Fy0 = cw.Dy0 * sin(cw.Cy * atan(cw.By0 * alpha_eq - cw.Ey * (cw.By0 * alpha_eq - atan(cw.By0 * alpha_eq))))
    Fy = (sigma_y / sigma) * (mu_y / cw.mu_y0) * (Fz / cw.Fz0) * Fy0
    return Fx, Fy


def wheel_force_to_car_force(u: MX, wheel_idx: int, Fx_w: MX, Fy_w: MX):
    delta = get_symbol(u, "delta{}".format(wheel_idx))
    cos_delta, sin_delta = cos(delta), sin(delta)
    Fx = Fx_w * cos_delta - Fy_w * sin_delta
    Fy = Fx_w * sin_delta + Fy_w * cos_delta
    return Fx, Fy


def car_force_to_car_torque(c: CarConstants, wheel_idx: int, Fx: MX, Fy: MX):
    ox, oy = c.wheel_constants[wheel_idx].x, c.wheel_constants[wheel_idx].y
    return -oy * Fx + ox * Fy
