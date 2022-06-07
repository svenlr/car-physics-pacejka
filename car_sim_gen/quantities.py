from casadi import MX


class CarPhysicalQuantities:
    def __init__(self, n_wheels):
        self.wheel_quantities = [WheelPhysicalQuantities() for _ in range(n_wheels)]


class WheelPhysicalQuantities:
    def __init__(self):
        pass
