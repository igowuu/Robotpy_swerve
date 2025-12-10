# LemonBot – Swerve Drive Robot (Python)

This repository contains the Python-based robot code for LemonBot, a swerve drive robot using WPILib, and CTRE Phoenix 6. The project supports both real-robot deployment and PyFRC simulation, including gyro and module simulation with full odometry support.

MagicBot is currently in use for our program, but other modules like TimedRobot can easily be implemented given our flexible structure.

## Features

### Swerve Drive (4-Module)
- Field-relative control
- Steering and drive PID with feedforward
- CANcoder absolute position reading
- Slew rate limiting for smooth driver control

### Teleop Control
- Dual-joystick input
- Deadband and speed ramping for smooth motion

### Sensors
- Pigeon2 gyro (real + simulated)
- CANcoder steering angle feedback

### Simulation (PyFRC)
- Wheel speeds, steer angles, and motor positions simulated
- Gyro + odometry updated in simulation
- Realistic motion tracking using ChassisSpeeds

## Code Structure

```
robot.py          – MagicBot robot container
drivetrain.py     – Swerve drive system (kinematics/odometry)
swervemodule.py   – Swerve module control (drive + steer)
physics.py        – PyFRC physics simulation engine
```

## Requirements
requirements.txt is included, though it has quite a few more packages than necessary. The needed resources are the following:

- Python 3.10+
- RobotPy WPILib
- robotpy-ctre (Phoenix 6)
- robotpy-halsim + pyfrc (for simulation)
- Proper CAN configuration for real hardware

## Test Code

```
robotpy test
```

## Run Simulation

```
robotpy sim
```

## Deploy to Robot

```
robotpy deploy
```
