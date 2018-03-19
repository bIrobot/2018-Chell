from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state
from components import navx_drive
from networktables import NetworkTables


class MiddlePosition(StatefulAutonomous):
    MODE_NAME = "Middle Position"

    def initialize(self):
        self.navx = navx_drive.Navx(self.navxSensor)
        self.sd = NetworkTables.getTable("SmartDashboard")

    @timed_state(duration=0.5, next_state='rotate', first=True)
    def drive_wait(self):
        self.navx.reset()
        self.gameData1 = self.sd.getString("gameData1", "No Data")
        if self.gameData1 == "Right":
            StatefulAutonomous.next_state(self, name='right_rotate')
        elif self.gameData1 == "Left":
            StatefulAutonomous.next_state(self, name='left_rotate')
        self.drive.arcadeDrive(0, 0, squaredInputs=False)

    @timed_state(duration=1.75, next_state='right_drive_forward')
    def right_rotate(self):
        self.drive.arcadeDrive(0.3, self.navx.drive(0.07, 0.3, 60), squaredInputs=False)

    @timed_state(duration=2.8, next_state='stop')
    def right_drive_forward(self):
        self.drive.arcadeDrive(0.23, self.navx.drive(0.07, 0.2, 0), squaredInputs=False)  # Drive forward and straight

    @timed_state(duration=1.75, next_state='left_drive_forward')
    def left_rotate(self):
        self.drive.arcadeDrive(0.3, self.navx.drive(0.07, 0.3, -60), squaredInputs=False)

    @timed_state(duration=2.8, next_state='stop')
    def left_drive_forward(self):
        self.drive.arcadeDrive(0.23, self.navx.drive(0.07, 0.2, 0), squaredInputs=False)  # Drive forward and straight

    @state()
    def stop(self):
        self.drive.arcadeDrive(0, 0, squaredInputs=False)
