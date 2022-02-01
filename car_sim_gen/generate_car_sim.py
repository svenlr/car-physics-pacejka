import os
import shutil
import numpy as np

from acados_template import AcadosSim, AcadosSimSolver

from car_sim_gen.car_model import create_car_model


def generate_car_sim(code_export_dir, time_step=0.001, num_stages=1, num_steps=1):
    model, c, q = create_car_model()
    sim = AcadosSim()
    sim.model = model

    sim.solver_options.T = time_step
    sim.solver_options.integrator_type = "ERK"
    sim.solver_options.num_stages = num_stages
    sim.solver_options.num_steps = num_steps
    sim.code_export_directory = code_export_dir

    Fzs = [cw.Fz0 for cw in c.wheel_constants]
    mu_x0s = [cw.mu_x0 for cw in c.wheel_constants]
    mu_y0s = [cw.mu_y0 for cw in c.wheel_constants]
    sim.parameter_values = np.array(Fzs + mu_x0s + mu_y0s)

    return AcadosSimSolver(sim, json_file=os.path.join(sim.code_export_directory, "acados_sim.json"))


if __name__ == '__main__':
    top_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    generate_car_sim(os.path.join(top_dir, "generated"))
