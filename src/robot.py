#!/usr/bin/env python3

import wpilib
from wpilib.drive import DifferentialDrive
from robotpy_ext.autonomous import AutonomousModeSelector
from robotpy_ext.common_drivers import navx
import ctre
from networktables import NetworkTables


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
        self.elevator = ctre.wpi_talonsrx.WPI_TalonSRX(5)  # Talon SRX controller with CAN address 5
        self.intakeLeft = wpilib.VictorSP(2)
        self.battleAxe = wpilib.VictorSP(3)
        self.actuator = wpilib.Spark(4)
        self.axeExtender = wpilib.Spark(5)

        ######################################

        self.encoderTicksPerInch = 1159

        self.elevator.setQuadraturePosition(0, 0)
        self.elevator.configForwardSoftLimitThreshold(int(round(-0.25 * self.encoderTicksPerInch)), 10)
        self.elevator.configReverseSoftLimitThreshold(int(round(-40 * self.encoderTicksPerInch)), 10)
        self.elevator.configForwardSoftLimitEnable(True, 10)
        self.elevator.configReverseSoftLimitEnable(True, 10)
        self.elevator.configPeakOutputForward(0.8, 10)
        self.elevator.configPeakOutputReverse(-1, 10)

        self.elevator.set(ctre.ControlMode.Position, 0)
        self.elevator.selectProfileSlot(0, 0)
        self.elevator.config_kF(0, 0, 10)
        self.elevator.config_kP(0, 0.6, 10)
        self.elevator.config_kI(0, 0.003, 10)
        self.elevator.config_kD(0, 0, 10)
        self.elevator.config_IntegralZone(0, 100, 10)

        # initialize limit switches and hall-effect sensors
        self.actuatorSwitchMin = wpilib.DigitalInput(0)
        self.actuatorSwitchMax = wpilib.DigitalInput(1)
        self.battleAxeSwitchUp = wpilib.DigitalInput(2)
        self.battleAxeSwitchDown = wpilib.DigitalInput(3)
        self.battleAxeExtenderSwitch = wpilib.DigitalInput(4)

        self.powerDistributionPanel = wpilib.PowerDistributionPanel()
        self.powerDistributionPanel.resetTotalEnergy()

        #
        # Communicate w/navX MXP via the MXP SPI Bus.
        # - Alternatively, use the i2c bus.
        # See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details
        #

        self.navxSensor = navx.AHRS.create_spi()
        # self.navxSensor = navx.AHRS.create_i2c()

        # Items in this dictionary are available in your autonomous mode
        # as attributes on your autonomous object
        self.components = {
            'drive': self.drive,
            'navxSensor': self.navxSensor,
            'actuator': self.actuator,
            'actuatorSwitchMin': self.actuatorSwitchMin,
            'actuatorSwitchMax': self.actuatorSwitchMax,
            'elevator': self.elevator,
            'intakeRight': self.intakeRight,
            'intakeLeft': self.intakeLeft,
            'gameData': self.driverStation.getInstance()
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

        self.elevator.setQuadraturePosition(0, 0)

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        self.automodes.run()

    def teleopInit(self):
        wpilib.IterativeRobot.teleopInit(self)

        self.setRamp = False
        self.rampState = False

        self.battleAxeUp = 0
        self.battleAxeDown = 0

        self.actuatorSpeedyIn = 0
        self.actuatorSpeedyOut = 0.3
        self.actuatorCount = 0

        self.startClimb = False
        self.climbMode = False
        self.climbRobot = False

        self.enableSequence1 = True
        self.enableSequence2 = True

        self.navxSensor.reset()

        self.minPosition = -0.25
        self.drivePosition = -11
        self.climbPosition = -32
        self.maxPosition = -40
        self.positionSelector = 1

        self.powerDistributionPanel.resetTotalEnergy()

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        rightXAxis = self.stick.getRawAxis(4) * 0.8
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

        self.sdUpdate()

        leftYAxis = self.stick.getRawAxis(1)  # Get joystick value
        leftYAxis = self.normalize(leftYAxis, 0.15)  # Set deadzone

        if -0.15 < self.stick.getRawAxis(1) < 0.15 and self.climbMode is False:
            if self.elevator.getQuadraturePosition() < (self.drivePosition + 1) * self.encoderTicksPerInch:
                self.enableSequence1 = False
            self.positionSelector = 2
        elif -0.15 < self.stick.getRawAxis(1) < 0.15 and self.climbMode is True:
            self.positionSelector = 3

            self.battleAxeUp = self.stick.getRawAxis(2) * -0.35
            self.battleAxeDown = self.stick.getRawAxis(3) * 0.35
            if self.battleAxeSwitchUp.get() is True:
                self.battleAxeUp = 0
            if self.battleAxeSwitchDown.get() is True:
                self.battleAxeDown = 0
            self.battleAxe.set(self.battleAxeUp + self.battleAxeDown)
        else:
            self.positionSelector = 0

        if self.stick.getRawAxis(2) > 0.1 and self.climbMode is False:
            self.positionSelector = 1
            self.intakeRight.set(-1)
            self.intakeLeft.set(-1)
        else:
            self.intakeRight.set(0)
            self.intakeLeft.set(0)

        if self.stick.getRawAxis(3) > 0.1 and self.climbMode is False:
            self.intakeRight.set(self.stick.getRawAxis(3))
            self.intakeLeft.set(self.stick.getRawAxis(3))

        if self.climbRobot is True:
            self.positionSelector = 2

        if self.positionSelector is 0:
            self.elevator.set(leftYAxis)
        elif self.positionSelector is 1:
            self.elevator.set(ctre.ControlMode.Position, int(round(self.minPosition * self.encoderTicksPerInch)))
        elif self.positionSelector is 2:
            self.elevator.set(ctre.ControlMode.Position, int(round(self.drivePosition * self.encoderTicksPerInch)))
        elif self.positionSelector is 3:
            self.elevator.set(ctre.ControlMode.Position, int(round(self.climbPosition * self.encoderTicksPerInch)))
        elif self.positionSelector is 4:
            self.elevator.set(ctre.ControlMode.Position, int(round(self.maxPosition * self.encoderTicksPerInch)))
        else:
            self.elevator.set(ctre.ControlMode.PercentOutput, 0)

        if self.stick.getRawButton(1) is True and self.stick.getRawButton(7) is True:  # A and start button
            self.startClimb = True

        if self.stick.getRawButton(3) is True:  # X button
            if self.actuatorCount < 20 and self.elevator.getQuadraturePosition() < \
                    (self.climbPosition + 1) * self.encoderTicksPerInch:
                self.actuatorSpeedyIn = -0.5
                self.actuatorSpeedyOut = 0
            else:
                self.actuatorSpeedyIn = 0
                self.actuatorSpeedyOut = 0
            self.actuatorCount += 1
        elif self.stick.getRawButton(3) is False and self.actuatorCount is not 0:
            self.actuatorSpeedyIn = 0
            self.actuatorSpeedyOut = 0.3
            self.actuatorCount = 0

        if self.actuatorSwitchMin.get() is False:
            self.actuatorSpeedyIn = 0
            self.climbMode = True
            if self.climbMode is False and self.enableSequence1 is False and self.enableSequence2 is False and\
                    self.startClimb is False:
                self.battleAxeUp = -0.3

        if self.actuatorSwitchMax.get() is False:
            self.actuatorSpeedyOut = 0
            if self.climbMode is False:
                self.battleAxeDown = 0.2

        self.actuator.set(self.actuatorSpeedyIn + self.actuatorSpeedyOut)

        if self.stick.getRawButton(2) is True and self.stick.getRawButton(7) is True and self.climbMode is True:
            self.climbRobot = True

        if self.startClimb is True:
            self.battleAxeUp = -0.35
            self.battleAxeDown = 0

        if self.battleAxeSwitchUp.get() is True:
            self.battleAxeUp = 0
            if self.startClimb is True:
                self.actuatorSpeedyIn = -0.4

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

        if self.battleAxeExtenderSwitch.get() is False:
            axeIn = 0
            self.enableSequence2 = False

        self.axeExtender.set(axeOut + axeIn)

    def testInit(self):
        pass
        # self.actuatorIn = 0
        # self.actuatorOut = 0

    def testPeriodic(self):
        """This function is called periodically during test mode."""
        # if self.actuatorSwitchMin.get() is False:
        #     actuatorIn = 0
        # if self.actuatorSwitchMax.get() is False:
        #     actuatorOut = 0
        # self.actuator.set(actuatorIn + actuatorOut)

        # self.battleAxe.set((self.stick.getRawAxis(2) * -0.5) + (self.stick.getRawAxis(3) * 0.5))

        self.sdUpdate()

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
        self.sd.putNumber('robot/time', wpilib.Timer.getMatchTime())
        self.sd.putNumber('drive/navx/yaw', self.navxSensor.getYaw())
        self.sd.putNumber('robot/elevator/encoder', self.elevator.getQuadraturePosition())
        self.sd.putNumber('robot/elevator/motorPercent', self.elevator.getMotorOutputPercent())
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
