from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state
from components import navx_drive
from networktables import NetworkTables


class LeftPosition(StatefulAutonomous):
    MODE_NAME = "Left Position"

    def initialize(self):
        self.navx = navx_drive.Navx(self.navxSensor)
        self.sd = NetworkTables.getTable("SmartDashboard")

    @timed_state(duration=0.5, next_state='drive_forward', first=True)
    def drive_wait(self):
        self.navx.reset()
        self.gameData2 = self.sd.getString("gameData2", "No Data")
        self.drive.arcadeDrive(0, 0, squaredInputs=False)

    @timed_state(duration=6, next_state='rotate')
    def drive_forward(self):
        if self.gameData2 == "Left":
            self.drive.arcadeDrive(0.3, self.navx.drive(self.sd.getNumber("slowSpeed", 0.07),
                                                         self.sd.getNumber("fastSpeed", 0.2), -3),
                                   squaredInputs=False)  # Drive forward and straight
        elif self.gameData2 == "Right":
            self.drive.arcadeDrive(0.2, self.navx.drive(self.sd.getNumber("slowSpeed", 0.07),
                                                         self.sd.getNumber("fastSpeed", 0.2), 0),
                                   squaredInputs=False)  # Drive forward and straight

    @timed_state(duration=2, next_state='drive_backward')
    def rotate(self):
        if self.gameData2 == "Left":
            self.drive.arcadeDrive(0, self.navx.drive(self.sd.getNumber("slowSpeed", 0.07),
                                                      self.sd.getNumber("fastSpeed", 0.2), 90),
                                   squaredInputs=False)
        elif self.gameData2 == "Right":
            self.drive.arcadeDrive(0, 0, squaredInputs=False)

    @timed_state(duration=1, next_state='stop')
    def drive_backward(self):
        if self.gameData2 == "Left":
            self.drive.arcadeDrive(-0.2, self.navx.drive(self.sd.getNumber("slowSpeed", 0.07),
                                                         self.sd.getNumber("fastSpeed", 0.2), 90),
                                   squaredInputs=False)
        elif self.gameData2 == "Right":
            self.drive.arcadeDrive(0, 0, squaredInputs=False)

    @state()
    def stop(self):
        self.drive.arcadeDrive(0, 0, squaredInputs=False)
