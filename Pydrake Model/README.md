# Pydrake Model

**Completed:**
- Basic .urdf and .xacro hexrotor description
- Linearized system about hover equilibrium
- Linear controllability determination about hover equilibirum
- Functioning simulation and visualization
- 

**Next Steps:**
- Implement LQR controller
- Implement trajectory tracking for controller testing
- 

**As Needed:**
- Update .urdf to better match physical hexrotor
- Implement more .xacro capabilities to automate .urdf generation
- 

## Overview
This directory includes the current Pydrake model for dynamical modeling and control. The current objective is to linearize the system about the hover equilibrium and establish a simple LQR controller for testing and simulation. More advanced geometric control will be implemented later, as well as a C++ Drake model.

## Contents
- **HexRotorDrake.py:** main script for model
- **/models:**
    - *HexRotor.urdf.xacro:* .urdf.xacro file for the hexrotor
- **/utils:** 
    - *control.py:* scripts for the controller design and implementation
    - *linear.py:* scripts for model linearization
    - *xacro.py:* scripts for .xacro commands
- 

## PYDRAKE CONFIGURATION:
First, you must run the following command in terminal to establish pydrake environment variable prior to running script (taken from [Drake: Installation via APT](https://drake.mit.edu/apt.html)):
```bash
export PATH="/opt/drake/bin${PATH:+:${PATH}}"
export PYTHONPATH="/opt/drake/lib/python$(python3 -c 'import sys; print("{0}.{1}".format(*sys.version_info))')/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
```
- Note: this is for standard Ubuntu WSL installation/implementation, it may differ for other distros.