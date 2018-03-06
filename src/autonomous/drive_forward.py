from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state


class DriveForward(StatefulAutonomous):
    MODE_NAME = "Drive Forward"

    DEFAULT = True

    def initialize(self):
        pass

    @state(first=True)
    def first(self):
        self.next_state('drive_forward')

    @timed_state(duration=3, next_state='stop')
    def drive_forward(self):
        self.drive.arcadeDrive(0.25, 0, squaredInputs=False)

    @state()
    def stop(self):
        self.drive.arcadeDrive(0, 0, squaredInputs=False)
