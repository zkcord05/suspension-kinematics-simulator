# MATLAB Suspension & Steering Kinematics Simulator

A MATLAB-based front suspension and steering analysis tool for a double-wishbone suspension system, built as a personal project to explore vehicle dynamics and suspension geometry.

## Features
- Models suspension hardpoints and steering geometry to calculate camber, toe, KPI, caster, scrub radius, mechanical trail, instant centers, and roll center height
- Implements bump-travel and rack-travel sweeps to evaluate camber gain, bump steer, and steering sensitivity
- Generates 3D suspension visualization and analysis plots to support suspension design decisions

## Files
- `main.m` — Entry point, runs full analysis and generates all plots
- `calcGeometry.m` — Computes suspension geometry from hardpoints
- `hardpoints.m` — Defines suspension hardpoint coordinates
- `bumpSweep.m` / `rackSweep.m` — Sweep analyses for bump and steering travel
- `camberGainSweep.m` — Camber gain analysis
- `plotSuspension.m` / `plotBumpSweep.m` / `plotCamberGain.m` / `plotRackSweep.m` — Visualization functions
- `solveLBJBump.m` / `solveUBJBump.m` / `solveSteeringArmAngle.m` — Constraint solvers

## Tools Used
- MATLAB

## Author
Zachary Cord — Mechanical Engineering, Colorado State University (Class of 2028)
