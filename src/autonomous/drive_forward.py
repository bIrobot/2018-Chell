from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state
from components import navx_drive
from networktables import NetworkTables


class DriveForward(StatefulAutonomous):
    MODE_NAME = "Drive Forward"

    DISABLED = True

    def initialize(self):
        self.navx = navx_drive.Navx(self.navxSensor)
        self.sd = NetworkTables.getTable("SmartDashboard")

    @timed_state(duration=0.5, next_state='drive_forward', first=True)
    def drive_wait(self):
        self.navx.reset()
        self.drive.arcadeDrive(0, 0, squaredInputs=False)

    @timed_state(duration=4, next_state='stop')
    def drive_forward(self):
        self.drive.arcadeDrive(0.25, self.navx.drive(0.07, 0.2, 0), squaredInputs=False)  # Drive forward and straight

    @state()
    def stop(self):
        self.drive.arcadeDrive(0, 0, squaredInputs=False)
