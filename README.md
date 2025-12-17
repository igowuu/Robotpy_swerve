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

## Requirements
requirements.txt is included for all packages.

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
