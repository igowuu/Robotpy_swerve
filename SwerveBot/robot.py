import wpilib
import wpimath
import wpimath.filter
import drivetrain

class MyRobot(wpilib.TimedRobot):
    """Main bot method"""
    def robotInit(self) -> None:
        self.lstick = wpilib.Joystick(0)
        self.rstick = wpilib.Joystick(1)

        self.drivetrain = drivetrain.Drivetrain()

        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)

        self.xSpeed = 0
        self.ySpeed = 0
        self.rot = 0

    def autonomousInit(self):
        self.timer = wpilib.Timer()
        self.timer.start()
        self.currentStep = 0
        self.sideTime = 0.1

    def autonomousPeriodic(self):
        self.xSpeed = 0
        self.ySpeed = 0
        self.rot = 0

        t = self.timer.get()
        totalTime = 4 * self.sideTime

        t_mod = t % totalTime

        if t_mod < self.sideTime:
            self.xSpeed = 0
            self.ySpeed = 50
        elif t_mod < 2*self.sideTime:
            self.xSpeed = 50
            self.ySpeed = 0
        elif t_mod < 3*self.sideTime:
            self.xSpeed = 0
            self.ySpeed = -50
        else:
            self.xSpeed = -50
            self.ySpeed = 0

    def teleopPeriodic(self) -> None:
        self.driveWithJoystick(True)
        self.drivetrain.updateOdometry()

    def driveWithJoystick(self, fieldRelative: bool) -> None:
        self.xSpeed = (
            -self.xspeedLimiter.calculate(
                wpimath.applyDeadband(self.lstick.getY(), 0.02)
            )
            * self.drivetrain.MAX_SPEED
        )

        self.ySpeed = (
            -self.yspeedLimiter.calculate(
                wpimath.applyDeadband(self.lstick.getX(), 0.02)
            )
            * self.drivetrain.MAX_SPEED
        )

        self.rot = (
            -self.rotLimiter.calculate(
                wpimath.applyDeadband(self.rstick.getX(), 0.02)
            )
            * self.drivetrain.MAX_ANGULAR_SPEED
        )
        

        self.drivetrain.drive(self.xSpeed, self.ySpeed, self.rot, fieldRelative, self.getPeriod())

