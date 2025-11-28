import math

import wpilib
import wpimath.controller
import wpimath.geometry
import wpimath.kinematics
import wpimath.trajectory

wheelRadius = 0.0508
encoderResolution = 4096
moduleMaxRotationalSpeed = math.pi
moduleMaxAngularAcceleration = math.tau # Full rotation

class Constants:
    # Drive controller:
    DRIVE_KP = 1
    DRIVE_KI = 0
    DRIVE_KD = 0
    # Turn controller:
    TURN_KP = 1
    TURN_KI = 0
    TURN_KD = 0

class SwerveModule:
    def __init__(
                self,
                driveMotorChannel: int,
                turningMotorChannel: int,
                driveEncoderChannelA: int,
                driveEncoderChannelB: int,
                turningEncoderChannelA: int,
                turningEncoderChannelB: int,
            ) -> None:
        
        self.driveMotor = wpilib.PWMSparkMax(driveMotorChannel)
        self.turningMotor = wpilib.PWMSparkMax(turningMotorChannel)

        self.driveEncoder = wpilib.Encoder(driveEncoderChannelA, driveEncoderChannelB)
        self.turningEncoder = wpilib.Encoder(
            turningEncoderChannelA, turningEncoderChannelB
        )

        self.drivePIDController = wpimath.controller.PIDController(
            Constants.DRIVE_KP, Constants.DRIVE_KI, Constants.DRIVE_KD
        )

        self.turningPIDController = wpimath.controller.ProfiledPIDController(
            Constants.TURN_KP, 
            Constants.TURN_KI, 
            Constants.TURN_KD,
            wpimath.trajectory.TrapezoidProfile.Constraints(
                moduleMaxRotationalSpeed,
                moduleMaxAngularAcceleration,
            ),
        )

        # Feedforward = how much voltage the motor *should* need
        self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 3)
        self.turnFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 0.5)

        self.driveEncoder.setDistancePerPulse(
            math.tau * wheelRadius / encoderResolution
        )

        self.turningEncoder.setDistancePerPulse(math.tau / encoderResolution)

        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        """Returns the current state of the module (wheel speed and wheel angle)."""
        return wpimath.kinematics.SwerveModuleState(
            self.driveEncoder.getRate(),
            wpimath.geometry.Rotation2d(self.turningEncoder.getDistance()),
        )

    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        """Returns the current position of the module (how far traveled and wheel angle since reset)"""
        return wpimath.kinematics.SwerveModulePosition(
            self.driveEncoder.getDistance(),
            wpimath.geometry.Rotation2d(self.turningEncoder.getDistance()),
        )

    def setDesiredState(self, desiredState: wpimath.kinematics.SwerveModuleState) -> None:
        """Sets the desired state for the module with speed and angle."""
        encoderRotation = wpimath.geometry.Rotation2d(self.turningEncoder.getDistance())

        # Optimize the reference state to avoid spinning further than 90 degrees
        desiredState.optimize(encoderRotation)

        desiredState.cosineScale(encoderRotation)

        driveOutput = self.drivePIDController.calculate(
            self.driveEncoder.getRate(), desiredState.speed
        )

        driveFeedforward = self.driveFeedforward.calculate(desiredState.speed)

        turnOutput = self.turningPIDController.calculate(
            self.turningEncoder.getDistance(), desiredState.angle.radians()
        )

        turnFeedforward = self.turnFeedforward.calculate(
            self.turningPIDController.getSetpoint().velocity
        )

        self.driveMotor.setVoltage(driveOutput + driveFeedforward)
        self.turningMotor.setVoltage(turnOutput + turnFeedforward)