from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state
from components import navx_drive
from networktables import NetworkTables


class MiddlePosition(StatefulAutonomous):
    MODE_NAME = "Middle Position"

    def initialize(self):
        self.tilt = False
        self.elevatorDown = False
        self.stop = False

        self.navx = navx_drive.Navx(self.navxSensor)
        self.sd = NetworkTables.getTable("SmartDashboard")

    @timed_state(duration=7, next_state='drive_wait', first=True)
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
            self.elevator.set(-0.55)
        elif self.elevatorSwitchDriveLow.get() is True and self.elevatorSwitchDriveHigh.get() is True and \
                self.elevatorDown is True:
            self.elevator.set(0.3)
        else:
            self.elevatorDown = True
            self.elevator.set(0)

        self.drive.arcadeDrive(0, 0, squaredInputs=False)

    @timed_state(duration=0.5, next_state='rotate')
    def drive_wait(self):
        self.navx.reset()
        self.gameData1 = self.sd.getString("gameData1", "No Data")
        if self.gameData1 == "Right":
            StatefulAutonomous.next_state(self, name='right_rotate')
        elif self.gameData1 == "Left":
            StatefulAutonomous.next_state(self, name='left_rotate')


        if self.elevatorSwitchDriveLow.get() is False:
            self.elevator.set(-0.4)
        elif self.elevatorSwitchDriveHigh.get() is False:
            self.elevator.set(0)
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)
        self.drive.arcadeDrive(0, 0, squaredInputs=False)

    @timed_state(duration=1.75, next_state='right_drive_forward')
    def right_rotate(self):
        self.drive.arcadeDrive(0.32, self.navx.drive(0.15, 0.5, 60), squaredInputs=False)


        if self.elevatorSwitchDriveLow.get() is False:
            self.elevator.set(-0.4)
        elif self.elevatorSwitchDriveHigh.get() is False:
            self.elevator.set(0)
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)

    @timed_state(duration=2.8, next_state='shoot')
    def right_drive_forward(self):
        self.drive.arcadeDrive(0.32, self.navx.drive(0.15, 0.5, 0), squaredInputs=False)  # Drive forward and straight


        if self.elevatorSwitchDriveLow.get() is False:
            self.elevator.set(-0.4)
        elif self.elevatorSwitchDriveHigh.get() is False:
            self.elevator.set(0)
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)

    @timed_state(duration=1.75, next_state='left_drive_forward')
    def left_rotate(self):
        self.drive.arcadeDrive(0.5, self.navx.drive(0.07, 0.5, -60), squaredInputs=False)


        if self.elevatorSwitchDriveLow.get() is False:
            self.elevator.set(-0.4)
        elif self.elevatorSwitchDriveHigh.get() is False:
            self.elevator.set(0)
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)

    @timed_state(duration=2.8, next_state='shoot')
    def left_drive_forward(self):
        self.drive.arcadeDrive(0.23, self.navx.drive(0.07, 0.2, 0), squaredInputs=False)  # Drive forward and straight


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
        self.drive.arcadeDrive(-0.2, self.navx.drive(0.07, 0.2, 0), squaredInputs=False)


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
