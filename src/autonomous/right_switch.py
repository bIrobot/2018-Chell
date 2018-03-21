from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state
from components import navx_drive
from networktables import NetworkTables


class RightSwitch(StatefulAutonomous):
    MODE_NAME = "Right Switch"

    def initialize(self):
        self.tilt = False
        self.elevatorDown = False
        self.stop = False

        self.navx = navx_drive.Navx(self.navxSensor)
        self.sd = NetworkTables.getTable("SmartDashboard")

    @timed_state(duration=4.5, next_state='drive_wait', first=True)
    def unlatch_and_tilt(self):
        if self.actuatorSwitchMin.get() is False:
            self.tilt = True
        if self.actuatorSwitchMax.get() is False:
            self.tilt = False
        if self.tilt is True and self.actuatorSwitchMax.get() is True:
            self.actuator.set(0.3)
        else:
            self.actuator.set(0)

        if self.elevatorSwitchClimbLow.get() is True and self.elevatorSwitchClimbHigh.get() is True and \
                self.elevatorDown is False:
            self.elevator.set(-0.7)
        elif self.elevatorSwitchDriveLow.get() is True and self.elevatorSwitchDriveHigh.get() is True and \
                self.elevatorDown is True:
            self.elevator.set(0.5)
        else:
            self.elevatorDown = True
            self.elevator.set(0)

        self.drive.arcadeDrive(0, 0, squaredInputs=False)

    @timed_state(duration=0.5, next_state='drive_forward')
    def drive_wait(self):
        self.navx.reset()
        self.gameData1 = self.sd.getString("gameData1", "No Data")
        if self.gameData1 == "Right":
            StatefulAutonomous.next_state(self, name='switch_drive_forward')
        else:
            StatefulAutonomous.next_state(self, name='cross_the_line')
        self.drive.arcadeDrive(0, 0, squaredInputs=False)

    @timed_state(duration=4, next_state='switch_rotate')
    def switch_drive_forward(self):
        self.drive.arcadeDrive(0.36, self.navx.drive(0.15, 0.5, 0), squaredInputs=False)


        if self.elevatorSwitchDriveLow.get() is False:
            self.elevator.set(-0.4)
        elif self.elevatorSwitchDriveHigh.get() is False:
            self.elevator.set(0)
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)

    @timed_state(duration=2, next_state='shoot')
    def switch_rotate(self):
        self.drive.arcadeDrive(0.2, self.navx.drive(0.15, 0.5, -90), squaredInputs=False)


        if self.elevatorSwitchDriveLow.get() is False:
            self.elevator.set(-0.4)
        elif self.elevatorSwitchDriveHigh.get() is False:
            self.elevator.set(0)
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)

    @timed_state(duration=1, next_state='drive_backward')
    def shoot(self):
        self.intakeRight.set(0.6)
        self.intakeLeft.set(0.6)
        self.actuator.set(0)
        self.drive.arcadeDrive(0, 0, squaredInputs=False)

        if self.elevatorSwitchDriveLow.get() is False:
            self.elevator.set(-0.4)
        elif self.elevatorSwitchDriveHigh.get() is False:
            self.elevator.set(0)

    @timed_state(duration=1, next_state='stop')
    def drive_backward(self):
        self.drive.arcadeDrive(-0.2, self.navx.drive(0.07, 0.2, -90), squaredInputs=False)

        if self.elevatorSwitchDriveLow.get() is False:
            self.elevator.set(-0.4)
        elif self.elevatorSwitchDriveHigh.get() is False:
            self.elevator.set(0)
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)

    @timed_state(duration=4, next_state='stop')
    def cross_the_line(self):
        self.drive.arcadeDrive(0.25, self.navx.drive(0.07, 0.2, 0), squaredInputs=False)


        if self.elevatorSwitchDriveLow.get() is False:
            self.elevator.set(-0.4)
        elif self.elevatorSwitchDriveHigh.get() is False:
            self.elevator.set(0)
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)

    @state()
    def stop(self):
        if self.elevatorSwitchDriveLow.get() is False:
            self.elevator.set(-0.4)
        elif self.elevatorSwitchDriveHigh.get() is False:
            self.elevator.set(0)
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)
        self.drive.arcadeDrive(0, 0, squaredInputs=False)
