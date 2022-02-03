import os
import shutil
import numpy as np

from acados_template import AcadosSim, AcadosSimSolver

from car_model import create_car_model
from generate_cpp_utils import generate_model_structs


def generate_car_sim(code_export_dir, time_step=0.001, num_stages=1, num_steps=1):
    model, c, q = create_car_model()
    sim = AcadosSim()
    sim.model = model

    sim.solver_options.T = time_step
    sim.solver_options.integrator_type = "ERK"
    sim.solver_options.num_stages = num_stages
    sim.solver_options.num_steps = num_steps
    sim.code_export_directory = code_export_dir

    Fz0s = [cw.Fz0 for cw in c.wheel_constants]
    mu_x0s = [cw.mu_x0 for cw in c.wheel_constants]
    mu_y0s = [cw.mu_y0 for cw in c.wheel_constants]
    sim.parameter_values = np.array(Fz0s + mu_x0s + mu_y0s)

    return AcadosSimSolver(sim, json_file=os.path.join(sim.code_export_directory, "acados_sim.json"))


if __name__ == '__main__':
    top_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sim_solver = generate_car_sim(os.path.join(top_dir, "generated"))
    model = sim_solver.sim_struct.model
    model_types_h = generate_model_structs(model, sim_solver.sim_struct.solver_options.T, "car_sim",
                                           default_params=sim_solver.sim_struct.parameter_values)
    try:
        with open(os.path.join(top_dir, "generated", "car_model_types.h")) as f:
            update_model_types_h = f.read() != model_types_h
    except FileNotFoundError:
        update_model_types_h = True
    if update_model_types_h:
        with open(os.path.join(top_dir, "generated", "car_model_types.h"), "w+") as f:
            f.write(model_types_h)
