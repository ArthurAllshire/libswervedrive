import math
import numpy as np
import time
from pyswervedrive import Chassis, Controller
import magicbot
import wpilib
import ctre

from components.swerve import SwerveChassis, SwerveModule, SwerveConfig

class SwerveTestRobot(magicbot.MagicRobot):
    lr_swerve: SwerveModule
    rr_swerve: SwerveModule
    lf_swerve: SwerveModule
    rf_swerve: SwerveModule
    chassis : SwerveChassis

    def createObjects(self):
        '''Create motors and stuff here'''
        self.lr_swerve_drive_motor = ctre.WPI_TalonSRX(11)
        self.rr_swerve_drive_motor = ctre.WPI_TalonSRX(21)
        self.lf_swerve_drive_motor = ctre.WPI_TalonSRX(31)
        self.rf_swerve_drive_motor = ctre.WPI_TalonSRX(41)
        self.lr_swerve_steer_motor = ctre.WPI_TalonSRX(12)
        self.rr_swerve_steer_motor = ctre.WPI_TalonSRX(22)
        self.lf_swerve_steer_motor = ctre.WPI_TalonSRX(32)
        self.rf_swerve_steer_motor = ctre.WPI_TalonSRX(42)
        self.lr_swerve_config = SwerveConfig(alpha=math.pi*3/4, l=1.414, r=0.05)
        self.rr_swerve_config = SwerveConfig(alpha=-math.pi*3/4, l=1.414, r=0.05)
        self.lf_swerve_config = SwerveConfig(alpha=math.pi*1/4, l=1.414, r=0.05)
        self.rf_swerve_config = SwerveConfig(alpha=-math.pi*1/4, l=1.414, r=0.05)

        self.joystick = wpilib.Joystick(0)

    def teleopInit(self):
        '''Called when teleop starts; optional'''
        pass

    def teleopPeriodic(self):
        '''Called on each iteration of the control loop'''
        # Read the joystick
        vx = -self.joystick.getY() * 3.0  # m/s
        vy = -self.joystick.getX() * 3.0
        vz = -self.joystick.getZ() * 0.5  # rad/s

        #self.chassis.drive(vx, vy, vz)
        self.chassis.icr_drive(vx, vy, vz)

if __name__ == '__main__':
    wpilib.run(SwerveTestRobot)