from robotpy_ext.autonomous import StatefulAutonomous, state
import ctre


class DoNothing(StatefulAutonomous):
    MODE_NAME = "Do Nothing"

    def initialize(self):
        self.tilt = False
        self.elevatorDown = False

        self.encoderTicksPerInch = 1159
        self.minPosition = -0.25
        self.drivePosition = -11
        self.climbPosition = -32
        self.maxPosition = -40

    @state(first=True)
    def stop(self):
        self.drive.arcadeDrive(0, 0, squaredInputs=False)
        self.elevator.set(ctre.ControlMode.Position, int(round(self.drivePosition * self.encoderTicksPerInch)))
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)
