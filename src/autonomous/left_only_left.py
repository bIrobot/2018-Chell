from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state
from components import navx_drive
from networktables import NetworkTables


class LeftOnlyLeft(StatefulAutonomous):
    MODE_NAME = "Left Only Left"

    def initialize(self):
        self.navx = navx_drive.Navx(self.navxSensor)
        self.sd = NetworkTables.getTable("SmartDashboard")

    @timed_state(duration=0.5, next_state='drive_forward', first=True)
    def drive_wait(self):
        self.navx.reset()
        self.gameData1 = self.sd.getString("gameData1", "No Data")
        self.gameData2 = self.sd.getString("gameData2", "No Data")
        if self.gameData2 == "Left":
            StatefulAutonomous.next_state(self, name='scale_drive_forward')
        elif self.gameData1 == "Left":
            StatefulAutonomous.next_state(self, name='switch_drive_forward')
        else:
            StatefulAutonomous.next_state(self, name='cross_the_line')
        self.drive.arcadeDrive(0, 0, squaredInputs=False)

    @timed_state(duration=5, next_state='scale_rotate')
    def scale_drive_forward(self):
        self.drive.arcadeDrive(0.3, self.navx.drive(0.07, 0.2, -5), squaredInputs=False)  # Drive forward at slight angle

    @timed_state(duration=2, next_state='scale_drive_backward')
    def scale_rotate(self):
        self.drive.arcadeDrive(0.2, self.navx.drive(0.07, 0.2, 90), squaredInputs=False)

    @timed_state(duration=1, next_state='stop')
    def scale_drive_backward(self):
        self.drive.arcadeDrive(-0.2, self.navx.drive(0.07, 0.2, 90), squaredInputs=False)

    @timed_state(duration=3.6, next_state='switch_rotate')
    def switch_drive_forward(self):
        self.drive.arcadeDrive(0.25, self.navx.drive(0.07, 0.2, 0), squaredInputs=False)

    @timed_state(duration=3, next_state='stop')
    def switch_rotate(self):
        self.drive.arcadeDrive(0.1, self.navx.drive(0.07, 0.2, 90), squaredInputs=False)

    @timed_state(duration=4, next_state='stop')
    def cross_the_line(self):
        self.drive.arcadeDrive(0.25, self.navx.drive(0.07, 0.2, 0), squaredInputs=False)

    @state()
    def stop(self):
        self.drive.arcadeDrive(0, 0, squaredInputs=False)
