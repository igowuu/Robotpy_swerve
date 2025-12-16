import math

from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpimath.controller import PIDController, SimpleMotorFeedforwardMeters
from phoenix6.hardware import TalonFX, CANcoder

import constants.feedforward as ff
from constants.hardware import WHEEL_RADIUS_M, GEAR_RATIO, MAX_VOLTAGE

from utils.conversions import Conversions

STEER_PID_MIN = -math.pi
STEER_PID_MAX = math.pi

class SwerveModule:
    def __init__(self, drive_id: int, steer_id: int, cancoder_id: int) -> None:
        self.drive_motor = TalonFX(drive_id, canbus="can0")
        self.steer_motor = TalonFX(steer_id, canbus="can0")
        self.cancoder = CANcoder(cancoder_id, canbus="can0")

        self.steer_pid = PIDController(ff.STEER_KP, ff.STEER_KI, ff.STEER_KD)
        self.drive_pid = PIDController(ff.DRIVE_KP, ff.DRIVE_KI, ff.DRIVE_KD)

        self.drive_ff = SimpleMotorFeedforwardMeters(ff.DRIVE_KS, ff.DRIVE_KV, ff.DRIVE_KA)

        self.steer_pid.enableContinuousInput(STEER_PID_MIN, STEER_PID_MAX)
        self.drive_motor.set_position(0)

    def _clamp(self, value: float, min_val: float, max_val: float) -> float:
        return max(min(value, max_val), min_val)

    def _read_cancoder_angle_radians(self) -> float:
        rotations = self.cancoder.get_absolute_position().value
        return Conversions.rotations_to_angle_rad(rotations)
    
    def _apply_steer_control(self, desired_angle: float):
        current_angle = self._read_cancoder_angle_radians()
        pid_voltage = self.steer_pid.calculate(current_angle, desired_angle)

        voltage = self._clamp(pid_voltage, -MAX_VOLTAGE, MAX_VOLTAGE)
        self.steer_motor.setVoltage(voltage)
    
    def get_position(self) -> SwerveModulePosition:
        """Returns the total drive position and rotation of the wheels."""
        rotor_rotations = self.drive_motor.get_position().value
        wheel_rotations = Conversions.rotor_rotations_to_wheel_rotations(rotor_rotations, GEAR_RATIO)
        drive_position_m = Conversions.wheel_rotations_to_meters(wheel_rotations, WHEEL_RADIUS_M)

        angle_rad = self._read_cancoder_angle_radians()
        return SwerveModulePosition(drive_position_m, Rotation2d(angle_rad))

    def set_desired_state(self, desired_state: SwerveModuleState) -> None:
        """Set the wheel speed and angle using feedforward and PID control."""
        current_angle = Rotation2d(self._read_cancoder_angle_radians())
        desired_state.optimize(current_angle)

        desired_speed_mps = desired_state.speed
        
        if abs(desired_speed_mps) < 0.01:
            return

        ff_voltage = self.drive_ff.calculate(desired_speed_mps)

        current_rotor_rate = self.drive_motor.get_velocity().value
        current_wheel_rate_mps = Conversions.rotor_rps_to_wheel_mps(
            rotor_rps=current_rotor_rate,
            gear_ratio=GEAR_RATIO,
            wheel_radius_m=WHEEL_RADIUS_M,
        )
        
        pid_voltage = self.drive_pid.calculate(current_wheel_rate_mps, desired_speed_mps)

        total_voltage = ff_voltage + pid_voltage
        total_voltage = self._clamp(total_voltage, -MAX_VOLTAGE, MAX_VOLTAGE)
        self.drive_motor.setVoltage(total_voltage)


        self._apply_steer_control(desired_state.angle.radians())
