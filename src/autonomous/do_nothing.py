from robotpy_ext.autonomous import StatefulAutonomous, state


class DoNothing(StatefulAutonomous):
    MODE_NAME = "Do Nothing"

    def initialize(self):
        self.tilt = False

    @state(first=True)
    def nothing(self):
        if self.actuatorSwitchMin.get() is False:
            self.tilt = True
        if self.actuatorSwitchMax.get() is False:
            self.tilt = False
        if self.tilt is True and self.actuatorSwitchMax.get() is True:
            self.actuator.set(0.3)
        else:
            self.actuator.set(0)
        self.drive.arcadeDrive(0, 0, squaredInputs=False)
