from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state
from components import navx_drive
from networktables import NetworkTables
import ctre


class RightOnlyRight(StatefulAutonomous):
    MODE_NAME = "Right Only Right"

    def initialize(self):
        self.tilt = False
        self.elevatorDown = False

        self.encoderTicksPerInch = 1159
        self.minPosition = -0.25
        self.drivePosition = -11
        self.climbPosition = -32
        self.maxPosition = -40

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

        if self.elevator.getQuadraturePosition() < (self.climbPosition + 1) * self.encoderTicksPerInch:
            self.elevatorDown = True
        if self.elevatorDown is False:
            self.elevator.set(ctre.ControlMode.Position, int(round(self.climbPosition * self.encoderTicksPerInch)))
        else:
            self.elevator.set(ctre.ControlMode.Position, int(round(self.drivePosition * self.encoderTicksPerInch)))

        self.drive.arcadeDrive(0.4, self.navx.drive(0.15, 0.5, -5), squaredInputs=False)
        self.intakeRight.set(0)
        self.intakeLeft.set(0)

    @timed_state(duration=0.5, next_state='cross_the_line')
    def drive_wait(self):
        self.gameData1 = self.sd.getString("gameData1", "No Data")
        self.gameData2 = self.sd.getString("gameData2", "No Data")

        self.data = self.gameData.getGameSpecificMessage()
        if self.data[1] == "R":
            StatefulAutonomous.next_state(self, name='scale_drive_forward')
        elif self.data[0] == "R":
            StatefulAutonomous.next_state(self, name='switch_drive_forward')
        else:
            StatefulAutonomous.next_state(self, name='cross_the_line')

        self.drive.arcadeDrive(0.5, self.navx.drive(0.15, 0.5, -5), squaredInputs=False)
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)

    @timed_state(duration=2.5, next_state='scale_rotate')
    def scale_drive_forward(self):
        self.drive.arcadeDrive(0.6, self.navx.drive(0.15, 0.5, 5), squaredInputs=False)
        self.elevator.set(ctre.ControlMode.Position, int(round(self.climbPosition * self.encoderTicksPerInch)))
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)

    @timed_state(duration=4, next_state='scale_shoot')
    def scale_rotate(self):
        self.drive.arcadeDrive(0, self.navx.drive(0.15, 0.6, -90), squaredInputs=False)
        self.elevator.set(ctre.ControlMode.Position, int(round(self.climbPosition * self.encoderTicksPerInch)))
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)

    @timed_state(duration=1, next_state='scale_drive_backward')
    def scale_shoot(self):
        self.drive.arcadeDrive(0, 0, squaredInputs=False)
        self.elevator.set(ctre.ControlMode.Position, int(round(self.climbPosition * self.encoderTicksPerInch)))
        self.intakeRight.set(0.6)
        self.intakeLeft.set(0.6)
        self.actuator.set(0)

    @timed_state(duration=1, next_state='scale_elevator_down')
    def scale_drive_backward(self):
        self.drive.arcadeDrive(-0.2, self.navx.drive(0.15, 0.5, -90), squaredInputs=False)
        self.elevator.set(ctre.ControlMode.Position, int(round(self.climbPosition * self.encoderTicksPerInch)))
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)
        self.drive.arcadeDrive(0, 0, squaredInputs=False)

    @timed_state(duration=5, next_state='stop')
    def scale_elevator_down(self):
        self.elevator.set(ctre.ControlMode.Position, int(round(self.drivePosition * self.encoderTicksPerInch)))
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)
        self.drive.arcadeDrive(0, 0, squaredInputs=False)

    @timed_state(duration=4, next_state='switch_rotate')
    def switch_drive_forward(self):
        self.drive.arcadeDrive(0.36, self.navx.drive(0.15, 0.5, 0), squaredInputs=False)

        self.elevator.set(ctre.ControlMode.Position, int(round(self.drivePosition * self.encoderTicksPerInch)))
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)

    @timed_state(duration=2, next_state='switch_shoot')
    def switch_rotate(self):
        self.drive.arcadeDrive(0.2, self.navx.drive(0.15, 0.5, -90), squaredInputs=False)
        self.elevator.set(ctre.ControlMode.Position, int(round(self.drivePosition * self.encoderTicksPerInch)))
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)

    @timed_state(duration=1, next_state='switch_drive_backward')
    def switch_shoot(self):
        self.drive.arcadeDrive(0, 0, squaredInputs=False)
        self.elevator.set(ctre.ControlMode.Position, int(round(self.drivePosition * self.encoderTicksPerInch)))
        self.intakeRight.set(0.6)
        self.intakeLeft.set(0.6)
        self.actuator.set(0)

    @timed_state(duration=1, next_state='stop')
    def switch_drive_backward(self):
        self.drive.arcadeDrive(-0.2, self.navx.drive(0.07, 0.2, -90), squaredInputs=False)
        self.elevator.set(ctre.ControlMode.Position, int(round(self.drivePosition * self.encoderTicksPerInch)))
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)

    @timed_state(duration=4, next_state='stop')
    def cross_the_line(self):
        self.drive.arcadeDrive(0.36, self.navx.drive(0.15, 0.5, 0), squaredInputs=False)
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
