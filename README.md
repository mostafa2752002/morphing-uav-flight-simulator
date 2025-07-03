
# Morphing Wing UAV Flight Simulator (Fixed-Wing Baseline Model)

This repository contains a custom-built MATLAB/Simulink flight simulator developed as part of a 3rd-year aerospace engineering project. The simulator models the full 6-DOF nonlinear dynamics of a **fixed-wing UAV** and serves as a foundational tool in our broader project on **morphing wing UAVs for aerial surveillance and reconnaissance**.

> ⚠️ **Note:** This simulator does not include morphing dynamics directly. It represents a baseline fixed-wing aircraft model, which we used as a platform for further analysis, controller design, and simulation within our morphing UAV project.

## 📌 Project Purpose

The main goals of this simulator are:
- To model and simulate realistic UAV flight dynamics using MATLAB and Simulink.
- To enable full-flight simulations by integrating equations of motion, forces/moments models, and control inputs.
- To support analysis of trim conditions, stability, and autopilot development for fixed-wing UAVs.
- To provide a foundation for testing control strategies related to morphing mechanisms (externally computed).



## 📁 Repository Structure

```
📦FlightSimulator/
├── README.md                 ← Project description
├── trim_solver.m             ← Trim condition solver using `fminsearch`
├── cost_function.m           ← Cost function for trim optimization
├── UAV_Morphing_MODEL.m      ← Full nonlinear 6-DOF UAV model
├── parameters_init.m         ← UAV parameters and constants initialization
├── trim_results.mat          ← Saved trim results (states & inputs)
├── <YourSimulinkModel>.slx   ← The Simulink model file
```



## ✈️ UAV Model Overview

The UAV is modeled with:
- 6 DOF nonlinear equations of motion
- Aerodynamic force and moment models (longitudinal and lateral-directional)
- Control surfaces: elevator, aileron, rudder, and throttle
- Propulsion modeled via simple quadratic thrust mapping
- Wind and gust disturbances (user-definable)
- No Aerospace Blockset used



## ⚙️ Features

- ✅ Full nonlinear simulation in Simulink
- ✅ Trim condition calculation for steady level flight
- ✅ Modular MATLAB code for easy tuning and testing
- ✅ Compatible with custom controllers and trajectory tracking scripts



## 🚀 How to Use

1. **Clone or download** the repository.
2. Open `parameters.m` and run it to initialize UAV constants.
3. Run `main.m` to compute the trim state and control inputs.
4. Analyze the results from the workspace or using your custom post-processing scripts.



## 🔬 Future Work

- Integrate the morphing camber mechanism using Simscape or Lookup Tables.
- Add a higher-fidelity propulsion model (e.g., based on motor maps).
- Implement real-time visualization of flight trajectories.
- Extend model to include autopilot and control allocation.



## 📚 References

- Beard & McLain, *Small Unmanned Aircraft: Theory and Practice*
- Course materials from AER 3510 - Flight Dynamics and Control
- Custom research and implementation on morphing wing mechanisms



## 🧑‍💻 Authors

- **Mustafa Muhammad, Wafaa Waheed** – System Modeling, MATLAB, Simulink
- Aerospace Engineering Department, Cairo University



