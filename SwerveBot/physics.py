import math
import typing

from pyfrc.physics.core import PhysicsInterface
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds

from constants.hardware import WHEEL_RADIUS_M, GEAR_RATIO

if typing.TYPE_CHECKING:
    from robot import LemonBot

class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "LemonBot"):
        self.physics_controller = physics_controller
        self.robot = robot

        self.drivetrain = robot.drivetrain
        self.modules = self.drivetrain.modules

        self.gyro_sim = self.drivetrain.gyro.sim_state

        self.pose = Pose2d()

        self._drive_rotor_positions = [0.0 for _ in self.modules]

        self._meters_per_rotor_rev = (
            2 * math.pi * WHEEL_RADIUS_M
        ) / GEAR_RATIO

        self.drivetrain.gyro.reset()

    def update_sim(self, _: float, tm_diff: float) -> None:
        chassis_speeds = ChassisSpeeds(
            self.drivetrain.xSpeed,
            self.drivetrain.ySpeed,
            self.drivetrain.omega,
        )

        if self.drivetrain.field_oriented:
            chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                self.drivetrain.xSpeed, 
                self.drivetrain.ySpeed, 
                self.drivetrain.omega, 
                self.drivetrain.gyro.getRotation2d()
            )
        else:
            chassis_speeds = ChassisSpeeds(
                self.drivetrain.xSpeed, 
                self.drivetrain.ySpeed, 
                self.drivetrain.omega
            )

        self.pose = self.physics_controller.drive(chassis_speeds, tm_diff)

        module_states = self.drivetrain.kinematics.toSwerveModuleStates(
            chassis_speeds
        )

        for i, (module, state) in enumerate(zip(self.modules, module_states)):
            wheel_speed_mps = state.speed
            delta_rotations = (
                wheel_speed_mps * tm_diff
            ) / self._meters_per_rotor_rev

            self._drive_rotor_positions[i] += delta_rotations

            module.drive_motor.sim_state.set_raw_rotor_position(
                self._drive_rotor_positions[i]
            )

            steer_angle_rad = state.angle.radians()
            steer_rotations = steer_angle_rad / (2 * math.pi)

            module.steer_motor.sim_state.set_raw_rotor_position(steer_rotations)
            module.cancoder.sim_state.set_raw_position(steer_rotations)

        self.gyro_sim.set_raw_yaw(self.pose.rotation().degrees())

        self.drivetrain.update_odometry()