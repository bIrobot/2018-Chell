from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state
from components import navx_drive
from networktables import NetworkTables


class LeftOnlyLeft(StatefulAutonomous):
    MODE_NAME = "Left Only Left"

    def initialize(self):
        self.tilt = False
        self.elevatorDown = False
        self.stop = False

        self.navx = navx_drive.Navx(self.navxSensor)
        self.navx.reset()
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
            # self.elevatorDown = True
            self.elevator.set(0)

        self.drive.arcadeDrive(0.4, self.navx.drive(0.15, 0.5, -5),
                                   squaredInputs=False)  # Drive forward at slight angle

    @timed_state(duration=0.5, next_state='drive_forward')
    def drive_wait(self):
        self.gameData1 = self.sd.getString("gameData1", "No Data")
        self.gameData2 = self.sd.getString("gameData2", "No Data")
        if self.gameData2 == "Left":
            StatefulAutonomous.next_state(self, name='scale_drive_forward')
        elif self.gameData1 == "Left":
            StatefulAutonomous.next_state(self, name='switch_drive_forward')
        else:
            StatefulAutonomous.next_state(self, name='cross_the_line')

        self.drive.arcadeDrive(0.5, self.navx.drive(0.15, 0.5, -5),
                                   squaredInputs=False)  # Drive forward at slight angle

    @timed_state(duration=2.5, next_state='scale_rotate')
    def scale_drive_forward(self):
        self.drive.arcadeDrive(0.6, self.navx.drive(0.15, 0.5, -5), squaredInputs=False)  # Drive forward at slight angle

        # if self.elevatorSwitchDriveLow.get() is False:
        #     self.elevator.set(-0.4)
        # elif self.elevatorSwitchDriveHigh.get() is False:
        #     self.elevator.set(0)
        self.elevator.set(0)

        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)
        self.drive.arcadeDrive(0, 0, squaredInputs=False)

    @timed_state(duration=4, next_state='scale_shoot')
    def scale_rotate(self):
        self.drive.arcadeDrive(0, self.navx.drive(0.15, 0.6, 90), squaredInputs=False)

        # start going up
        if self.elevatorSwitchMax.get() is True:
            self.elevator.set(-0.7)
        else:
            self.elevator.set(0)
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)

    # @timed_state(duration=2, next_state='scale_shoot')
    # def scale_elevator_up(self):
    #     # finish going up
    #     if self.elevatorSwitchMax.get() is True:
    #         self.elevator.set(-0.55)
    #     else:
    #         self.elevator.set(0)
    #     self.intakeRight.set(0)
    #     self.intakeLeft.set(0)
    #     self.actuator.set(0)
    #     self.drive.arcadeDrive(0, 0, squaredInputs=False)

    @timed_state(duration=1, next_state='scale_drive_backward')
    def scale_shoot(self):
        self.intakeRight.set(0.6)
        self.intakeLeft.set(0.6)
        self.elevator.set(0)
        self.actuator.set(0)
        self.drive.arcadeDrive(0, 0, squaredInputs=False)

    @timed_state(duration=1, next_state='scale_elevator_down')
    def scale_drive_backward(self):
        self.drive.arcadeDrive(-0.2, self.navx.drive(0.15, 0.5, 90), squaredInputs=False)

        if self.elevatorSwitchDriveLow.get() is False:
            self.elevator.set(-0.4)
        elif self.elevatorSwitchDriveHigh.get() is False:
            self.elevator.set(0)
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)
        self.drive.arcadeDrive(0, 0, squaredInputs=False)

    @timed_state(duration=5, next_state='stop')
    def scale_elevator_down(self):
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

    @timed_state(duration=2, next_state='switch_shoot')
    def switch_rotate(self):
        self.drive.arcadeDrive(0.2, self.navx.drive(0.15, 0.5, 90), squaredInputs=False)

        if self.elevatorSwitchDriveLow.get() is False:
            self.elevator.set(-0.4)
        elif self.elevatorSwitchDriveHigh.get() is False:
            self.elevator.set(0)
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)

    @timed_state(duration=1, next_state='switch_drive_backward')
    def switch_shoot(self):
        self.intakeRight.set(0.6)
        self.intakeLeft.set(0.6)
        self.actuator.set(0)
        self.drive.arcadeDrive(0, 0, squaredInputs=False)

        if self.elevatorSwitchDriveLow.get() is False:
            self.elevator.set(-0.4)
        elif self.elevatorSwitchDriveHigh.get() is False:
            self.elevator.set(0)

    @timed_state(duration=1, next_state='stop')
    def switch_drive_backward(self):
        self.drive.arcadeDrive(-0.2, self.navx.drive(0.07, 0.2, 90), squaredInputs=False)

        if self.elevatorSwitchDriveLow.get() is False:
            self.elevator.set(-0.4)
        elif self.elevatorSwitchDriveHigh.get() is False:
            self.elevator.set(0)
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)

    @timed_state(duration=4, next_state='stop')
    def cross_the_line(self):
        self.drive.arcadeDrive(0.36, self.navx.drive(0.15, 0.5, 0), squaredInputs=False)

        if self.elevatorSwitchDriveLow.get() is False:
            self.elevator.set(-0.4)
        elif self.elevatorSwitchDriveHigh.get() is False:
            self.elevator.set(0)
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)
        self.drive.arcadeDrive(0, 0, squaredInputs=False)

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
