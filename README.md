## Detailed, fast C/C++ vehicle physics based on acados
The python code in this project is used to generate C and C++ libraries
that provide discretized car dynamics.
In the current state, one can simulate a tenth-scale chassis with drive-train
and tire friction dynamics.

### Features
- 4 explicitly simulated wheels, independent 4-wheel steering
- Pacejka-type combined tire slip model (Similarity Method)
- Changes of friction coefficients / vertical load during runtime
- Underactuated differential gear simulation
- Drive-train with DC motor model (single motor)

### How to integrate this in a multi-body simulator such as Gazebo?
Given the current velocities of a car body, vertical loads on the wheels
and the controls (throttle and steering angles), 
this software can calculate the movement of the car in the form of 
force and torque to be applied to the car within the next millisecond.

![Timing with multi-body simulation.png](doc/interaction_timing_diagram.png)

![Integration in multi-body simulation](doc/multi_body_interaction.png)

- Set up a rigid body for the car in the employed simulator (e.g. Gazebo)
- Model wheels as spheres so that there is only a single contact point
- Turn off friction for the wheels (friction will be handled by the generated software)
- In each update: 
  - Read the velocity states, vertical forces and controls from Gazebo.
  - Then, set those values as the current state for the car physics simulation.
  - Advance the car physics simulation.
  - Get force / torque and set it to the car body in Gazebo.

## Installation
### Prerequisites
- Install acados: https://github.com/acados/acados
- Install acados Python Interface: https://docs.acados.org/python_interface/index.html#installation
- Optional, for C++ demo: Python3 installation with numpy and matplotlib (dependency of matplotlibcpp)

### Build
The CMakeLists.txt is set up so that the code is automatically 
re-generated using acados when the Python files are changed.
```bash
mkdir -p build/
cd build/
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j6
```

### Use
The build output to be linked in a C project consists of the
`libacados_sim_solver_car.so` library.
Refer to the acados API documentation on how to use it.

The build output to be linked in a C++ project consists of the
`libcar_physics_pacejka.so` library (see CMakeLists.txt) and the
`libacados_sim_solver_car.so` library.
A simple C++ interface `CarSimulation.h/.cpp` is build into
the library `libcar_physics_pacejka.so` which handles the interaction
with the acados solver in the other library.


## Appendix

### Model
States: $v_x, v_y, r, \omega, \omega_{\triangle f}, \omega_{\triangle r}$  
Linear velocities, angular velocity, motor velocity, differential velocities
  
Controls: $DC, \delta_{fl},\delta_{fr},\delta_{rl},\delta_{rr}$  
Duty cycle, steering angles

#### Spatial view of the model
Spatial states: $v_x, v_y, r$  
![friction model](doc/spatial_model.png)

#### Drive-train view of the model
Drive-train states: $\omega, \omega_{\triangle f}, \omega_{\triangle r}$  
$\omega_{\triangle f}, \omega_{\triangle r}$ are the relative velocities 
in-between front wheels and in-between rear wheels respectively

![drive train model](doc/drive_train_model.png)

### Plots generated with the implemented Pacejka-type tire model
![tire model](doc/tire_model.png)