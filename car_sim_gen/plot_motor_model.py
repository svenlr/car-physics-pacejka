import numpy as np
from matplotlib import pyplot as plt
import casadi
from casadi import MX

from car_sim_gen.car_model import calc_motor_torque
from car_sim_gen.constants import CarConstants

if __name__ == '__main__':
    c = CarConstants()
    dc = MX.sym("dc")
    omega = MX.sym("omega")

    M = calc_motor_torque(c, dc, omega)
    calc_torque = casadi.Function("calc_torque", [dc, omega], [M], dict())

    omegas = np.linspace(0, 240)
    torques = []
    for omega_val in omegas:
        torque = calc_torque(1, omega_val)
        torques.append(torque)
    plt.plot(omegas, torques)
    plt.savefig("../doc/motor_model.png")
    plt.show()