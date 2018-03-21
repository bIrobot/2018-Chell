#!/usr/bin/env python3

import wpilib
from wpilib.drive import DifferentialDrive
import ctre
from robotpy_ext.common_drivers import navx
from networktables import NetworkTables
from robotpy_ext.autonomous import AutonomousModeSelector


class MyRobot(wpilib.IterativeRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        # joystick 1 on the driver station
        self.stick = wpilib.XboxController(0)

        self.driverStation = wpilib.DriverStation

        self.frontRight = ctre.wpi_talonsrx.WPI_TalonSRX(3)
        self.rearRight = ctre.wpi_talonsrx.WPI_TalonSRX(1)
        self.right = wpilib.SpeedControllerGroup(self.frontRight, self.rearRight)

        self.frontLeft = ctre.wpi_talonsrx.WPI_TalonSRX(4)
        self.rearLeft = ctre.wpi_talonsrx.WPI_TalonSRX(2)
        self.left = wpilib.SpeedControllerGroup(self.frontLeft, self.rearLeft)

        self.frontRight.setExpiration(0.2)
        self.rearRight.setExpiration(0.2)
        self.frontRight.setExpiration(0.2)
        self.rearLeft.setExpiration(0.2)

        self.drive = DifferentialDrive(self.left, self.right)

        # initialize motors other than drive
        self.intakeRight = wpilib.VictorSP(0)
        self.elevator = wpilib.VictorSP(1)
        self.intakeLeft = wpilib.VictorSP(2)
        self.battleAxe = wpilib.VictorSP(3)
        self.actuator = wpilib.Spark(4)
        self.axeExtender = wpilib.Spark(5)

        # initialize limit switches and hall-effect sensors
        self.actuatorSwitchMin = wpilib.DigitalInput(0)
        self.actuatorSwitchMax = wpilib.DigitalInput(1)
        self.battleAxeSwitchUp = wpilib.DigitalInput(2)
        self.battleAxeSwitchDown = wpilib.DigitalInput(3)
        self.battleAxeExtenderSwitch = wpilib.DigitalInput(4)
        self.elevatorSwitchMin = wpilib.DigitalInput(5)
        self.elevatorSwitchDriveLow = wpilib.DigitalInput(6)
        self.elevatorSwitchDriveHigh = wpilib.DigitalInput(7)
        self.elevatorSwitchClimbLow = wpilib.DigitalInput(8)
        self.elevatorSwitchClimbHigh = wpilib.DigitalInput(9)
        self.elevatorSwitchMax = wpilib.DigitalInput(10)

        self.powerDistributionPanel = wpilib.PowerDistributionPanel()
        self.powerDistributionPanel.resetTotalEnergy()

        #
        # Communicate w/navX MXP via the MXP SPI Bus.
        # - Alternatively, use the i2c bus.
        # See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details
        #

        self.navxSensor = navx.AHRS.create_spi()
        # self.navx = navx.AHRS.create_i2c()

        # Items in this dictionary are available in your autonomous mode
        # as attributes on your autonomous object
        self.components = {
            'drive': self.drive,
            'navxSensor': self.navxSensor,
            'actuator': self.actuator,
            'actuatorSwitchMin': self.actuatorSwitchMin,
            'actuatorSwitchMax': self.actuatorSwitchMax,
            'elevator': self.elevator,
            'elevatorSwitchMin': self.elevatorSwitchMin,
            'elevatorSwitchDriveLow': self.elevatorSwitchDriveLow,
            'elevatorSwitchDriveHigh': self.elevatorSwitchDriveHigh,
            'elevatorSwitchClimbLow': self.elevatorSwitchClimbLow,
            'elevatorSwitchClimbHigh': self.elevatorSwitchClimbHigh,
            'elevatorSwitchMax': self.elevatorSwitchMax,
            'intakeRight': self.intakeRight,
            'intakeLeft': self.intakeLeft
        }

        # * The first argument is the name of the package that your autonomous
        #   modes are located in
        # * The second argument is passed to each StatefulAutonomous when they
        #   start up
        self.automodes = AutonomousModeSelector('autonomous', self.components)

        NetworkTables.initialize()
        self.sd = NetworkTables.getTable("SmartDashboard")

        wpilib.CameraServer.launch('vision.py:main')

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        gameData = self.driverStation.getInstance().getGameSpecificMessage()
        if gameData[0] == 'L':
            self.sd.putString('gameData1', "Left")
        elif gameData[0] == 'R':
            self.sd.putString('gameData1', "Right")
        if gameData[1] == 'L':
            self.sd.putString('gameData2', "Left")
        elif gameData[1] == 'R':
            self.sd.putString('gameData2', "Right")
        if gameData[2] == 'L':
            self.sd.putString('gameData3', "Left")
        elif gameData[2] == 'R':
            self.sd.putString('gameData3', "Right")

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        self.automodes.run()

    def teleopInit(self):
        wpilib.IterativeRobot.teleopInit(self)

        self.setRamp = False
        self.rampState = False

        self.sdUpdateCount = 0

        self.battleAxeUp = 0
        self.battleAxeDown = 0

        self.actuatorSpeedyIn = 0
        self.actuatorSpeedyOut = 0.3
        self.elevatorDirection = 0
        self.actuatorBack = False

        self.elevatorSetpointPosition = -0.55
        self.elevatorClimbPosition = -0.45

        self.startClimb = False
        self.climbMode = False
        self.adjustMode = False
        self.climbRobot = False

        self.enableSequence1 = True
        self.enableSequence2 = True

        self.navxSensor.reset()

        self.powerDistributionPanel.resetTotalEnergy()

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        rightXAxis = self.stick.getRawAxis(4) * 0.65
        rightYAxis = self.stick.getRawAxis(5) * -1

        # rightXAxis = 0
        # rightYAxis = 0

        if rightYAxis >= 0.25 or rightYAxis <= -0.25:
            if rightYAxis <= -0.65:
                rightYAxis = -0.65
            self.setRamp = True
        else:
            self.setRamp = False

        if self.setRamp != self.rampState:
            if self.setRamp is True:
                self.frontRight.configOpenLoopRamp(1, 0)
                self.rearRight.configOpenLoopRamp(1, 0)
                self.frontRight.configOpenLoopRamp(1, 0)
                self.rearLeft.configOpenLoopRamp(1, 0)
                self.rampState = True
            else:
                self.frontRight.configOpenLoopRamp(0, 0)
                self.rearRight.configOpenLoopRamp(0, 0)
                self.frontRight.configOpenLoopRamp(0, 0)
                self.rearLeft.configOpenLoopRamp(0, 0)
                self.rampState = False

        self.drive.arcadeDrive(rightYAxis, rightXAxis, squaredInputs=True)

        if self.sdUpdateCount >= 5:
            self.sdUpdate()
            self.sdUpdateCount = 0
        self.sdUpdateCount += 1
        self.sd.putNumber('robot/time', wpilib.Timer.getMatchTime())

        leftYAxis = self.stick.getRawAxis(1)  # Get joystick value
        leftYAxis = self.normalize(leftYAxis, 0.15)  # Set deadzone

        if self.stick.getRawAxis(1) < 0.15 and self.stick.getRawAxis(1) > -0.15 and self.climbMode is False:
            if self.elevatorSetpointPosition < 0:
                elevatorUp = self.elevatorSetpointPosition
                elevatorDown = 0
            else:
                elevatorUp = 0
                elevatorDown = self.elevatorSetpointPosition
            if self.elevatorSwitchDriveHigh.get() is False:
                self.elevatorSetpointPosition = 0
            elif self.elevatorSwitchDriveLow.get() is False:
                self.elevatorSetpointPosition = -0.45
                self.enableSequence1 = False
                self.intakeRight.set(0)
                self.intakeLeft.set(0)
            if self.elevatorSwitchMin.get() is False:
                self.intakeRight.set(0)
                self.intakeLeft.set(0)
        elif self.stick.getRawAxis(1) < 0.15 and self.stick.getRawAxis(1) > -0.15 and self.climbMode is True:
            if self.elevatorClimbPosition < 0:
                elevatorUp = self.elevatorClimbPosition
                elevatorDown = 0
            else:
                elevatorUp = 0
                elevatorDown = self.elevatorClimbPosition
            if self.elevatorSwitchClimbHigh.get() is False:
                self.elevatorClimbPosition = 0
            elif self.elevatorSwitchClimbLow.get() is False:
                self.elevatorClimbPosition = -0.6
            self.battleAxeUp = self.stick.getRawAxis(2) * -0.35
            self.battleAxeDown = self.stick.getRawAxis(3) * 0.35
            if self.battleAxeSwitchUp.get() is True:
                self.battleAxeUp = 0
            if self.battleAxeSwitchDown.get() is True:
                self.battleAxeDown = 0
            self.battleAxe.set(self.battleAxeUp + self.battleAxeDown)
        elif self.climbMode is False:
            if leftYAxis < 0:
                elevatorUp = leftYAxis
                elevatorDown = 0
                if self.elevatorSwitchDriveHigh.get() is False:
                    self.elevatorSetpointPosition = 0.3
            else:
                elevatorUp = 0
                elevatorDown = leftYAxis
                if self.elevatorSwitchDriveLow.get() is False:
                    self.elevatorSetpointPosition = -0.4
        else:
            if leftYAxis < 0:
                elevatorUp = leftYAxis
                elevatorDown = 0
                if self.elevatorSwitchClimbHigh.get() is False:
                    self.elevatorClimbPosition = 0.3
            else:
                elevatorUp = 0
                elevatorDown = leftYAxis
                if self.elevatorSwitchClimbLow.get() is False:
                    self.elevatorClimbPosition = -0.4

        if self.stick.getRawAxis(2) > 0.1 and self.climbMode is False:
            elevatorUp = 0
            elevatorDown = self.stick.getRawAxis(2)
            self.intakeRight.set(-1)
            self.intakeLeft.set(-1)
            if self.elevatorSwitchDriveLow.get() is False:
                self.elevatorSetpointPosition = -0.4
        else:
            self.intakeRight.set(0)
            self.intakeLeft.set(0)

        if self.climbRobot is True:
            if self.elevatorSetpointPosition < 0:
                elevatorUp = self.elevatorSetpointPosition
                elevatorDown = 0
            else:
                elevatorUp = 0
                elevatorDown = self.elevatorSetpointPosition
            if self.elevatorSwitchDriveHigh.get() is False:
                self.elevatorSetpointPosition = 0.4
            elif self.elevatorSwitchDriveLow.get() is False:
                self.elevatorSetpointPosition = 0

        if self.elevatorSwitchMin.get() is False:
            elevatorDown = 0

        if self.elevatorSwitchMax.get() is False:
            elevatorUp = 0

        self.elevator.set(elevatorUp + elevatorDown)

        if self.stick.getRawButton(2) is True:
            self.adjustMode = True

        if self.stick.getRawAxis(3) > 0.1 and self.climbMode is False:
            self.intakeRight.set(self.stick.getRawAxis(3))
            self.intakeLeft.set(self.stick.getRawAxis(3))

        if self.stick.getRawButton(1) is True and self.stick.getRawButton(7) is True:
            self.startClimb = True

        if self.actuatorSwitchMin.get() is False:
            self.actuatorSpeedyIn = 0
            self.actuatorBack = True
            if self.climbMode is False and self.adjustMode is False and self.enableSequence1 is False\
                    and self.enableSequence2 is False and self.startClimb is False:
                self.battleAxeUp = -0.3

        if self.actuatorSwitchMax.get() is False:
            self.actuatorSpeedyOut = 0
            if self.climbMode is False and self.adjustMode is False:
                self.battleAxeDown = 0.2

        self.actuator.set(self.actuatorSpeedyIn + self.actuatorSpeedyOut)

        if self.stick.getRawButton(2) is True and self.stick.getRawButton(7) is True and self.climbMode is True:
            self.climbRobot = True
            self.elevatorSetpointPosition = 0.4

        if self.startClimb is True:
            self.battleAxeUp = -0.35
            self.battleAxeDown = 0

        if self.battleAxeSwitchUp.get() is True:
            self.battleAxeUp = 0
            if self.startClimb is True:
                self.actuatorSpeedyIn = -0.4

        if self.actuatorBack is True:
            self.climbMode = True

        if self.battleAxeSwitchDown.get() is True:
            self.battleAxeDown = 0

        if self.climbMode is False:
            self.battleAxe.set(self.battleAxeUp + self.battleAxeDown)

        if self.stick.getRawButton(6) is True:
            axeOut = 1
            axeIn = 0
        elif self.stick.getRawButton(5) is True:
            axeOut = 0
            axeIn = -1
        else:
            axeOut = 0
            axeIn = 0

        if self.enableSequence2 is True:
            axeIn = -1

        if self.battleAxeExtenderSwitch.get() is True:
            axeIn = 0
            self.enableSequence2 = False

        self.axeExtender.set(axeOut + axeIn)

    def testPeriodic(self):
        """This function is called periodically during test mode."""

    def normalize(self, joystickInput, deadzone):
        """joystickInput should be between -1 and 1, deadzone should be between 0 and 1."""
        if joystickInput > 0:
            if (joystickInput - deadzone) < 0:
                return 0
            else:
                return (joystickInput - deadzone) / (1 - deadzone)
        elif joystickInput < 0:
            if (joystickInput + deadzone) > 0:
                return 0
            else:
                return (joystickInput + deadzone) / (1 - deadzone)
        else:
            return 0

    def sdUpdate(self):
        self.sd.putNumber('drive/navx/yaw', self.navxSensor.getYaw())
        self.sd.putNumber('robot/totalCurrent', self.powerDistributionPanel.getTotalCurrent())
        self.sd.putNumber('robot/totalEnergy', self.powerDistributionPanel.getTotalEnergy())
        self.sd.putNumber('robot/totalPower', self.powerDistributionPanel.getTotalPower())
        if wpilib.DriverStation.getInstance().getAlliance() is wpilib.DriverStation.Alliance.Red:
            theme = "red"
            self.sd.putString('theme', theme)
        elif wpilib.DriverStation.getInstance().getAlliance() is wpilib.DriverStation.Alliance.Blue:
            theme = "blue"
            self.sd.putString('theme', theme)


if __name__ == "__main__":
    wpilib.run(MyRobot)
