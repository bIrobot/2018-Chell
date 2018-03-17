from robotpy_ext.autonomous import StatefulAutonomous, state


class DoNothing(StatefulAutonomous):
    MODE_NAME = "Do Nothing"

    def initialize(self):
        pass

    @state(first=True)
    def nothing(self):
        self.drive.arcadeDrive(0, 0, squaredInputs=False)
