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
        self.drive.arcadeDrive(0, 0, squaredInputs=False)

    @timed_state(duration=1.75, next_state='drive_forward')
    def rotate(self):
        if self.gameData1 == "Left":
            self.drive.arcadeDrive(0.3, self.navx.drive(self.sd.getNumber("slowSpeed", 0.07),
                                                         self.sd.getNumber("fastSpeed", 0.3), -60),
                                   squaredInputs=False)
        elif self.gameData1 == "Right":
            self.drive.arcadeDrive(0.3, self.navx.drive(self.sd.getNumber("slowSpeed", 0.07),
                                                         self.sd.getNumber("fastSpeed", 0.3), 60),
                                   squaredInputs=False)

    @timed_state(duration=2.8, next_state='stop')
    def drive_forward(self):
        if self.gameData1 == "Left":
            self.drive.arcadeDrive(0.25, self.navx.drive(self.sd.getNumber("slowSpeed", 0.07),
                                                         self.sd.getNumber("fastSpeed", 0.2), 0),
                                   squaredInputs=False)  # Drive forward and straight
        elif self.gameData1 == "Right":
            self.drive.arcadeDrive(0.23, self.navx.drive(self.sd.getNumber("slowSpeed", 0.07),
                                                         self.sd.getNumber("fastSpeed", 0.2), 0),
                                   squaredInputs=False)  # Drive forward and straight

    @state()
    def stop(self):
        self.drive.arcadeDrive(0, 0, squaredInputs=False)
