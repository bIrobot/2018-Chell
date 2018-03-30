from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state
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

    @timed_state(duration=7, next_state='stop', first=True)
    def unlatch_and_tilt(self):
        if self.actuatorSwitchMin.get() is False:
            self.tilt = True
        if self.actuatorSwitchMax.get() is False:
            self.tilt = False
        if self.tilt is True and self.actuatorSwitchMax.get() is True:
            self.actuator.set(0.3)
        else:
            self.actuator.set(0)

        if self.elevator.getQuadraturePosition() < (self.climbPosition + 1) * self.encoderTicksPerInch:
            self.elevatorDown = True
        if self.elevatorDown is False:
            self.elevator.set(ctre.ControlMode.Position, int(round(self.climbPosition * self.encoderTicksPerInch)))
        else:
            self.elevator.set(ctre.ControlMode.Position, int(round(self.drivePosition * self.encoderTicksPerInch)))

        self.drive.arcadeDrive(0, 0, squaredInputs=False)
        self.intakeRight.set(0)
        self.intakeLeft.set(0)

    @timed_state(duration=7, next_state='shoot')
    def hold(self):
        self.drive.arcadeDrive(0, 0, squaredInputs=False)
        self.elevator.set(ctre.ControlMode.Position, int(round(self.drivePosition * self.encoderTicksPerInch)))
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)

    @timed_state(duration=1, next_state='up')
    def shoot(self):
        self.drive.arcadeDrive(0, 0, squaredInputs=False)
        self.elevator.set(ctre.ControlMode.Position, int(round(self.drivePosition * self.encoderTicksPerInch)))
        self.intakeRight.set(0.6)
        self.intakeLeft.set(0.6)
        self.actuator.set(0)

    @timed_state(duration=4, next_state='shoot_high')
    def up(self):
        self.drive.arcadeDrive(0, 0, squaredInputs=False)
        self.elevator.set(ctre.ControlMode.Position, int(round(self.climbPosition * self.encoderTicksPerInch)))
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)

    @timed_state(duration=1, next_state='down')
    def shoot_high(self):
        self.drive.arcadeDrive(0, 0, squaredInputs=False)
        self.elevator.set(ctre.ControlMode.Position, int(round(self.climbPosition * self.encoderTicksPerInch)))
        self.intakeRight.set(0.6)
        self.intakeLeft.set(0.6)
        self.actuator.set(0)

    @timed_state(duration=10, next_state='stop')
    def down(self):
        self.drive.arcadeDrive(0, 0, squaredInputs=False)
        self.elevator.set(ctre.ControlMode.Position, int(round(self.drivePosition * self.encoderTicksPerInch)))
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)

    @state()
    def stop(self):
        self.drive.arcadeDrive(0, 0, squaredInputs=False)
        self.elevator.set(ctre.ControlMode.Position, int(round(self.drivePosition * self.encoderTicksPerInch)))
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)
