from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state
from components import navx_drive
from networktables import NetworkTables
import ctre


class MiddlePosition(StatefulAutonomous):
    MODE_NAME = "Middle Position"

    def initialize(self):
        self.tilt = False
        self.elevatorDown = False

        self.encoderTicksPerInch = 1159
        self.minPosition = -0.25
        self.drivePosition = -11
        self.climbPosition = -32
        self.maxPosition = -40

        self.elevator.setQuadraturePosition(0, 0)

        self.navx = navx_drive.Navx(self.navxSensor)
        self.sd = NetworkTables.getTable("SmartDashboard")

    @timed_state(duration=5, next_state='drive_wait', first=True)
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

    @timed_state(duration=0.5, next_state='rotate')
    def drive_wait(self):
        self.navx.reset()
        self.gameData1 = self.sd.getString("gameData1", "No Data")
        if self.gameData1 == "Right":
            StatefulAutonomous.next_state(self, name='right_rotate')
        elif self.gameData1 == "Left":
            StatefulAutonomous.next_state(self, name='left_rotate')

        self.drive.arcadeDrive(0, 0, squaredInputs=False)
        self.elevator.set(ctre.ControlMode.Position, int(round(self.drivePosition * self.encoderTicksPerInch)))
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)

    @timed_state(duration=2, next_state='right_drive_forward')
    def right_rotate(self):
        self.drive.arcadeDrive(0.32, self.navx.drive(0.15, 0.5, 60), squaredInputs=False)
        self.elevator.set(ctre.ControlMode.Position, int(round(self.drivePosition * self.encoderTicksPerInch)))
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)

    @timed_state(duration=2.8, next_state='shoot')
    def right_drive_forward(self):
        self.drive.arcadeDrive(0.32, self.navx.drive(0.15, 0.5, 0), squaredInputs=False)  # Drive forward and straight
        self.elevator.set(ctre.ControlMode.Position, int(round(self.drivePosition * self.encoderTicksPerInch)))
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)

    @timed_state(duration=3, next_state='left_drive_forward')
    def left_rotate(self):
        self.drive.arcadeDrive(0.32, self.navx.drive(0.15, 0.5, -60), squaredInputs=False)
        self.elevator.set(ctre.ControlMode.Position, int(round(self.drivePosition * self.encoderTicksPerInch)))
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)

    @timed_state(duration=3, next_state='shoot')
    def left_drive_forward(self):
        self.drive.arcadeDrive(0.32, self.navx.drive(0.15, 0.5, 0), squaredInputs=False)  # Drive forward and straight
        self.elevator.set(ctre.ControlMode.Position, int(round(self.drivePosition * self.encoderTicksPerInch)))
        self.intakeRight.set(0)
        self.intakeLeft.set(0)
        self.actuator.set(0)

    @timed_state(duration=1, next_state='drive_backward')
    def shoot(self):
        self.drive.arcadeDrive(0, 0, squaredInputs=False)
        self.elevator.set(ctre.ControlMode.Position, int(round(self.drivePosition * self.encoderTicksPerInch)))
        self.intakeRight.set(0.6)
        self.intakeLeft.set(0.6)
        self.actuator.set(0)

    @timed_state(duration=1, next_state='stop')
    def drive_backward(self):
        self.drive.arcadeDrive(-0.2, self.navx.drive(0.07, 0.2, 0), squaredInputs=False)
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
