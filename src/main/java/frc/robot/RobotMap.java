// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class RobotMap {

    // CAN motor addresses
    public static final int kFrontLeftDriveCANSparkMaxMotor = 1;
    public static final int kBackLeftDriveCANSparkMaxMotor = 2;
    public static final int kBackRightDriveCANSparkMaxMotor = 3;
    public static final int kFrontRightDriveCANSparkMaxMotor = 4;
    public static final int kFrontLeftTurnCANSparkMaxMotor = 11;
    public static final int kBackLeftTurnCANSparkMaxMotor = 12;
    public static final int kBackRightTurnCANSparkMaxMotor = 13;
    public static final int kFrontRightTurnCANSparkMaxMotor = 14;
    public static final int kExteriorFeederVictorSpxMotor = 5;
    public static final int kArmRotatorSparkMaxMotor = 7;
    public static final int kArmExtensionSparkMaxMotor = 8;

    // PWM motor addresses
    public static final int kInteriorFeederSparkMotor = 0;
    public static final int kFlipperVacuumSparkMotor = 1;
    public static final int kGripVacuum1SparkMotor = 2;
    public static final int kGripVacuum2SparkMotor = 3;
    public static final int kGripVacuum3SparkMotor = 4;

    // REV Pneumatic Hub solenoid addresses
    public static final int kIntakeArm = 0;
    public static final int kConeOrientator = 1;
    public static final int kFlipperArm = 2;
    public static final int kIntakeVacuumRelease = 3;
    public static final int kArmVacuumRelease = 4;

    // Digital IO addresses
    public static final int kFrontLeftTurnPulseWidthDigIO = 1;
    public static final int kBackLeftTurnPulseWidthDigIO = 2;
    public static final int kBackRightTurnPulseWidthDigIO = 3;
    public static final int kFrontRightTurnPulseWidthDigIO = 4;
    public static final int kGamePeicePresentSwitch = 5;
    public static final int kConeAllInSwitch = 6;

    // Operator input USB ports
    public static final int kLeftJoystick = 0;
    public static final int kRightJoystick = 1;
    public static final int kButtonBox = 2;
    public static final int kXBoxController = 3;

    // Driver Buttons
    public static final int kToggleIntakeButton = 2;

    // Co-Piolet Button board
    public static final int kHighGamePeiceButton = 1;
    public static final int kMidGamePeiceButton = 2;
    public static final int kLowGamePeiceButton = 3;
    public static final int kArmReleaseButton = 4;
    public static final int kReturnArmToLoadingButton = 5;
    public static final int kIntakeReleaseButton = 6;

    // Zeroed values, should be in radians
    //private static final double kOffsetZeroed = Math.PI / 2;
    public static final double kZeroedFrontLeft = 2.672 + Math.toRadians(90);
    public static final double kZeroedFrontRight = 2.815 + Math.toRadians(90);
    public static final double kZeroedBackLeft = 0.908 + Math.toRadians(90);
    public static final double kZeroedBackRight = 1.683 + Math.toRadians(90);
}
