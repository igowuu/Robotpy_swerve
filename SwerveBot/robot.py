from components.drivetrain import DriveTrain
from dashboard import Dashboard

import magicbot

class LemonBot(magicbot.MagicRobot):
    def createObjects(self) -> None:
        self.drivetrain = DriveTrain()
        self.dashboard = Dashboard(self.drivetrain)

    def robotPeriodic(self):
        self.dashboard.execute()

    def teleopPeriodic(self) -> None:
        self.drivetrain.drive_with_joystick()
        self.drivetrain.update_odometry()