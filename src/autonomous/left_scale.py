from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state
from components import navx_drive
from networktables import NetworkTables


class LeftScale(StatefulAutonomous):
    MODE_NAME = "Left Scale"

    DISABLED = True

    def initialize(self):
        self.navx = navx_drive.Navx(self.navxSensor)
        self.sd = NetworkTables.getTable("SmartDashboard")

    @timed_state(duration=0.5, next_state='drive_forward', first=True)
    def drive_wait(self):
        self.navx.reset()
        self.gameData2 = self.sd.getString("gameData2", "No Data")
        if self.gameData2 == "Right":
            StatefulAutonomous.next_state(self, name='right_drive_forward')
        elif self.gameData2 == "Left":
            StatefulAutonomous.next_state(self, name='left_drive_forward')
        self.drive.arcadeDrive(0, 0, squaredInputs=False)

    @timed_state(duration=5, next_state='left_rotate')
    def left_drive_forward(self):
        self.drive.arcadeDrive(0.3, self.navx.drive(0.07, 0.2, -5), squaredInputs=False)  # Drive forward at slight angle

    @timed_state(duration=2, next_state='left_drive_backward')
    def left_rotate(self):
        self.drive.arcadeDrive(0.2, self.navx.drive(0.07, 0.2, 90), squaredInputs=False)

    @timed_state(duration=1, next_state='stop')
    def left_drive_backward(self):
        self.drive.arcadeDrive(-0.2, self.navx.drive(0.07, 0.2, 90), squaredInputs=False)

    @timed_state(duration=2.8, next_state='right_cross_field')
    def right_drive_forward(self):
        self.drive.arcadeDrive(0.35, self.navx.drive(0.07, 0.2, 0), squaredInputs=False)

    @timed_state(duration=3.4, next_state='right_turn_straight')
    def right_cross_field(self):
        self.drive.arcadeDrive(0.4, self.navx.drive(0.1, 0.3, 90), squaredInputs=False)

    @timed_state(duration=2.7, next_state='right_drive_backward')
    def right_turn_straight(self):
        self.drive.arcadeDrive(0.2, self.navx.drive(0.07, 0.2, -10), squaredInputs=False)

    @timed_state(duration=1, next_state='stop')
    def right_drive_backward(self):
        self.drive.arcadeDrive(-0.2, self.navx.drive(0.07, 0.2, -10), squaredInputs=False)

    @state()
    def stop(self):
        self.drive.arcadeDrive(0, 0, squaredInputs=False)
