from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state
from components import navx_drive
from networktables import NetworkTables


class MiddlePosition(StatefulAutonomous):
    MODE_NAME = "Middle Position"

    def initialize(self):
        self.navx = navx_drive.Navx(self.navxSensor)
        self.sd = NetworkTables.getTable("SmartDashboard")

    @timed_state(duration=0.5, next_state='drive_forward', first=True)
    def drive_wait(self):
        self.navx.reset()
        self.drive.arcadeDrive(0, 0, squaredInputs=False)

    @timed_state(duration=3, next_state='stop')
    def drive_forward(self):
        self.drive.arcadeDrive(0.25, self.navx.drive(self.sd.getNumber("slowSpeed", 0.07),
                                                     self.sd.getNumber("fastSpeed", 0.2), 0),
                               squaredInputs=False)  # Drive forward and straight

    @state()
    def stop(self):
        self.drive.arcadeDrive(0, 0, squaredInputs=False)
