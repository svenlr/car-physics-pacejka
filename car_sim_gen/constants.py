import numpy as np


class CarConstants:
    def __init__(self):
        # Physical parameters
        self.m = 1.9  # [kg] body mass
        self.I_z = 0.034  # inertia around Z
        self.Im = 1e-5  # motor and gear inertia

        l_f = 0.13  # [m] distance from center of gravity to front axle
        l_r = 0.13  # [m] distance from center of gravity to rear axle
        wheel_sep = 0.17  # [m] distance between left and right tires
        wheel_positions = init_4w_positions(l_f, l_r, wheel_sep)
        self.wheel_constants = [WheelConstants(pos[0], pos[1]) for pos in wheel_positions]

        self.mu_diff_visc = 0.001

        # drive train parameters

        self.Tm_p = 1.788  # [Nm] max torque by the motor at dc=1 and omega=0
        self.Tm_emv = 5.73352202e-03  # [Nm/s] torque reduction per rps
        self.Tm_drag = 6.38179494e-06  # [Nm/s^2] drag torque per rps * rps

    @property
    def n_wheels(self):
        return len(self.wheel_constants)


class WheelConstants:
    def __init__(self, pos_x, pos_y):
        self.x = pos_x  # relative x position to the CoG
        self.y = pos_y  # relative y position to the CoG
        self.I_w = 2.23e-5  # wheel inertia
        self.m_w = 0.05  # [kg] mass per wheel

        # Pacejka tire model parameters
        self.Fz0 = 3  # [N]
        self.a0 = 0.008  # [m]
        self.b = 0.007  # [m]
        self.radius = 0.02625  # [m] tire radius
        self.Cy = 1.3  # shape
        self.Ey = -1.0  # curvature
        self.Cz = 2.3  # shape
        self.Ez = -2.0  # curvature
        self.Cx = 1.5  # shape
        self.Ex = -1.0  # curvature
        self.mu_y0 = 1.0  # friction coefficient scaling
        self.mu_x0 = 1.0  # friction coefficient scaling
        self.c1 = 8.0
        self.c2 = 1.33
        self.c3 = 0.25
        self.c4 = 0.5
        self.c5 = 1.0
        self.c6 = 0.3
        self.c7 = 100.0
        self.c8 = 15.0
        self.c9 = 0.3
        self.c10 = 0.0
        self.c11 = 4.0

        # ==================
        # derived parameters
        # ==================
        c = self  # shortcut
        # default cornering stiffness
        self.C_Fa0 = c.c1 * c.c2 * c.Fz0 * np.sin(2 * np.arctan(1 / c.c2))
        # peak value at Fz=Fz0
        self.Dy0 = c.mu_y0 * c.Fz0
        self.By0 = c.C_Fa0 / (c.Cy * c.Dy0)
        # default longitudinal slip stiffness
        self.C_Fk0 = c.c8 * c.Fz0
        # peak value at Fz=Fz0
        self.Dx0 = c.mu_x0 * c.Fz0
        self.Bx0 = c.C_Fk0 / (c.Cx * c.Dx0)

        # print("C_Fa0", self.C_Fa0)
        # print("Dy0", self.Dy0)
        # print("By0", self.By0)
        # print("C_Fk0", self.C_Fk0)
        # print("Dx0", self.Dx0)
        # print("Bx0", self.Bx0)


def init_4w_positions(l_f, l_r, wheel_sep):
    return [
        [l_f, wheel_sep / 2], [l_f, -wheel_sep / 2],
        [-l_r, wheel_sep / 2], [-l_r, -wheel_sep / 2]
    ]
