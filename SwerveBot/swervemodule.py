import math
import wpimath
import wpimath.trajectory
from wpimath.controller import PIDController, ProfiledPIDController, SimpleMotorFeedforwardMeters
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from phoenix6.hardware import TalonFX, CANcoder

WHEEL_RADIUS = 0.0508
DRIVE_GEAR_RATIO = 6.75
FALCON_CPR = 2048.0

_METERS_PER_MOTOR_REV = (2.0 * math.pi * WHEEL_RADIUS) / DRIVE_GEAR_RATIO

DRIVE_kP = 0.6
DRIVE_kI = 0.0
DRIVE_kD = 0.0

TURN_kP = 3.0
TURN_kI = 0.0
TURN_kD = 0.0

DRIVE_kS = 0.17
DRIVE_kV = 0.104
DRIVE_kA = 0.01

TURN_kS = 0.14
TURN_kV = 0.375
TURN_kA = 0.0

class SwerveModule:
    """Initialization of hardware and their getters/setters."""
    def __init__(self, name: str, drive_id: int, steer_id: int, cancoder_id: int) -> None:
        self.name = name

        self.drive_motor: TalonFX = TalonFX(drive_id, canbus="can0")
        self.steer_motor: TalonFX = TalonFX(steer_id, canbus="can0")
        self.cancoder: CANcoder = CANcoder(cancoder_id, canbus="can0")

        self.drive_pid = PIDController(DRIVE_kP, DRIVE_kI, DRIVE_kD)
        self.turning_pid = ProfiledPIDController(
            TURN_kP,
            TURN_kI,
            TURN_kD,
            wpimath.trajectory.TrapezoidProfile.Constraints(math.tau, math.tau * 10),
        )

        self.turning_pid.enableContinuousInput(-math.pi, math.pi)

        self.drive_ff = SimpleMotorFeedforwardMeters(DRIVE_kS, DRIVE_kV, DRIVE_kA)
        self.turn_ff = SimpleMotorFeedforwardMeters(TURN_kS, TURN_kV, TURN_kA)

        self.last_desired_state = SwerveModuleState(0.0, Rotation2d(0.0))

        self.drive_motor.set_position(0)

    def getPosition(self) -> SwerveModulePosition:
        rotations = self.drive_motor.get_position().value 
        meters = rotations * _METERS_PER_MOTOR_REV
        angle = self._read_cancoder_angle_radians()

        return SwerveModulePosition(meters, Rotation2d(angle))

    def getState(self) -> SwerveModuleState:
        rotations_per_sec = self.drive_motor.get_velocity().value
        meters_per_sec = rotations_per_sec * _METERS_PER_MOTOR_REV
        angle = self._read_cancoder_angle_radians()
        return SwerveModuleState(meters_per_sec, Rotation2d(angle))

    def _read_cancoder_angle_radians(self) -> float:
        rotations = self.cancoder.get_absolute_position().value
        angle_rad = rotations * (2.0 * math.pi)
        return ((angle_rad + math.pi) % (2 * math.pi)) - math.pi

    def setDesiredState(self, desired_state: SwerveModuleState) -> None:
        """Sets the desired state for the module with speed and angle."""
        current_state = self.getState()
        current_speed = current_state.speed

        desired_speed = desired_state.speed

        drive_feedforward = self.drive_ff.calculate(desired_speed)
        drive_output = self.drive_pid.calculate(current_speed, desired_speed)

        drive_voltage = drive_output + drive_feedforward

        desired_angle = desired_state.angle.radians()
        
        turn_output = self.turning_pid.calculate(self._read_cancoder_angle_radians(), desired_angle)
        turn_feedforward = self.turn_ff.calculate(self.turning_pid.getSetpoint().velocity)
        turn_voltage = turn_output + turn_feedforward

        self.drive_motor.setVoltage(drive_voltage)


        self.steer_motor.setVoltage(turn_voltage)
