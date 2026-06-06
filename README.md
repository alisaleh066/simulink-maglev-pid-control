# MECH1320 Magnetic Levitation PID Control Simulation

A Simulink and MATLAB control-systems project modelling and controlling an electromagnetic ball levitation system.

The project uses nonlinear modelling, linearisation, state-space analysis, transfer functions, and PID controller tuning to simulate stable magnetic levitation.

## Project Structure

```text
src/
  analysis_code.m        MATLAB script for equilibrium points, linearisation, state-space checks, transfer functions, and PID analysis

models/
  final_project.slx      Main Simulink model
  final_project_pid.slx  PID-focused Simulink model
  segway.slx             Additional Simulink model
  solar_tracker.slx      Additional Simulink model

reports/
  final_report.pdf       Final project report
```

## Tools and Technologies

- MATLAB
- Simulink
- Control System Toolbox
- PID controller design
- State-space analysis
- Transfer function modelling
- Nonlinear system simulation

## Key Skills Demonstrated

- Modelling nonlinear electromechanical systems
- Linearising around an equilibrium point
- Analysing controllability and observability
- Designing and tuning PID controllers
- Building and testing Simulink models
- Technical report writing

## How to Use

1. Open MATLAB.
2. Open the `.slx` files inside the `models/` folder.
3. Run `src/analysis_code.m` to reproduce the symbolic/control-system calculations.
4. Refer to `reports/final_report.pdf` for the modelling assumptions and results.

## Notes

This repository was cleaned for portfolio use. Draft documents, assignment guideline PDFs, `.DS_Store`, `__MACOSX`, and generated MATLAB project metadata were removed.
