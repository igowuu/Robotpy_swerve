from wpilib import SmartDashboard
from robot import DriveTrain

class Dashboard:
    def __init__(self, drivetrain: DriveTrain):
        self.drivetrain = drivetrain

    def _robot_pose(self) -> str:
        pose = self.drivetrain.get_pose()
        return f"X: {pose.X():.2f} m, Y: {pose.Y():.2f} m, o: {pose.rotation().degrees():.1f}"

    def _field_oriented_mode(self) -> str:
        return str(self.drivetrain.field_oriented)

    def _gyro_heading(self) -> str:
        return f"{self.drivetrain.gyro.getRotation2d().degrees():.1f}"

    def _module_0(self) -> str:
        pos = self.drivetrain.modules[0].get_position()
        return f"Pos: {pos.distance:.2f} m, Angle: {pos.angle.degrees():.1f}"

    def _module_0_desired(self) -> str:
        state = self.drivetrain.get_module_states()[0]
        return f"Speed: {state.speed:.2f} m/s, Angle: {state.angle.degrees():.1f}"

    def execute(self) -> None:
        SmartDashboard.putString("Robot pose", self._robot_pose())
        SmartDashboard.putString("Field oriented", self._field_oriented_mode())
        SmartDashboard.putString("Gyro heading", self._gyro_heading())
        SmartDashboard.putString("SwerveMod 0", self._module_0())
        SmartDashboard.putString("SwerveMod 0 desired", self._module_0_desired())