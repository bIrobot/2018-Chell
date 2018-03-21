from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state


class DoNothing(StatefulAutonomous):
    MODE_NAME = "Do Nothing"

    def initialize(self):
        self.tilt = False
        self.elevatorDown = False
        self.stop = False

    @timed_state(duration=7, next_state='hold', first=True)
    def unlatch_and_tilt(self):
        if self.actuatorSwitchMin.get() is False:
            self.tilt = True
        if self.actuatorSwitchMax.get() is False:
            self.tilt = False
        if self.tilt is True and self.actuatorSwitchMax.get() is True:
            self.actuator.set(0.3)
        else:
            self.actuator.set(0)

        if self.elevatorSwitchClimbLow.get() is True and self.elevatorSwitchClimbHigh.get() is True and\
                self.elevatorDown is False:
            self.elevator.set(-0.55)
        elif self.elevatorSwitchDriveLow.get() is True and self.elevatorSwitchDriveHigh.get() is True and\
                self.elevatorDown is True:
            self.elevator.set(0.3)
        else:
            self.elevatorDown = True
            self.elevator.set(0)

        self.drive.arcadeDrive(0, 0, squaredInputs=False)

    @timed_state(duration=7, next_state='shoot')
    def hold(self):
        if self.elevatorSwitchDriveLow.get() is False:
            self.elevator.set(-0.4)
        elif self.elevatorSwitchDriveHigh.get() is False:
            self.elevator.set(0)
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)
        self.drive.arcadeDrive(0, 0, squaredInputs=False)

    @timed_state(duration=1, next_state='up')
    def shoot(self):
        self.intakeRight.set(0.6)
        self.intakeLeft.set(0.6)
        self.elevator.set(0)
        self.actuator.set(0)
        self.drive.arcadeDrive(0, 0, squaredInputs=False)

    @timed_state(duration=4, next_state='shoot_high')
    def up(self):
        if self.elevatorSwitchClimbLow.get() is True and self.elevatorSwitchClimbHigh.get() is True:
            self.elevator.set(-0.55)
        else:
            self.elevator.set(0)
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)
        self.drive.arcadeDrive(0, 0, squaredInputs=False)

    @timed_state(duration=1, next_state='down')
    def shoot_high(self):
        self.intakeRight.set(0.6)
        self.intakeLeft.set(0.6)
        self.elevator.set(0)
        self.actuator.set(0)
        self.drive.arcadeDrive(0, 0, squaredInputs=False)

    @timed_state(duration=10, next_state='stop')
    def down(self):
        if self.stop is False:
            if self.elevatorSwitchDriveLow.get() is True and self.elevatorSwitchDriveHigh.get() is True:
                self.elevator.set(0.3)
            else:
                self.stop = True
                self.elevator.set(0)
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)
        self.drive.arcadeDrive(0, 0, squaredInputs=False)

    @state()
    def stop(self):
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.elevator.set(0)
        self.actuator.set(0)
        self.drive.arcadeDrive(0, 0, squaredInputs=False)
