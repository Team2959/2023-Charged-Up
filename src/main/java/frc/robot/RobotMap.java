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
    public static final int kFlipperSparkMaxMotor = 9;

    // PWM motor addresses
    public static final int kInteriorFeederSparkMotor = 0;
    public static final int kFlipperVacuumSparkMotor = 1;
    public static final int kGripVacuumSparkMotor = 2;

    // REV Pneumatic Hub solenoid addresses
    public static final int kConeOrientater = 1;
    public static final int kIntakeArm = 2;
    public static final int kIntakeVacuumRelease = 3;
    public static final int kArmVacuumRelease1 = 4;
    public static final int kArmVacuumRelease2 = 5;
    public static final int kArmVacuumRelease3 = 6;
    public static final int kArmVacuumRelease4 = 7;

    // Digital IO addresses
    public static final int kFrontLeftTurnPulseWidthDigIO = 1;
    public static final int kBackLeftTurnPulseWidthDigIO = 2;
    public static final int kBackRightTurnPulseWidthDigIO = 3;
    public static final int kFrontRightTurnPulseWidthDigIO = 4;
    public static final int kGamePieceInSwitch = 5;
    public static final int kConeAllInSwitch = 6;
    public static final int kRopeEncoderDigIO = 7;
    public static final int kRotatorArmEncoderPulseWidthDIO = 8;
    public static final int kGamePieceDetectedSwitch = 9;

    // Operator input USB ports
    public static final int kLeftJoystick = 0;
    public static final int kRightJoystick = 1;
    public static final int kButtonBox = 2;
    public static final int kXBoxController = 3;

    // Driver Buttons
    public static final int kToggleIntakeButton = 2;

    // Co-Piolet Button board
    public static final int kHighGamePeiceButton = 8;
    public static final int kMidGamePeiceButton = 9;
    public static final int kLowGamePeiceButton = 12;
    public static final int kArmReleaseButton = 1;
    public static final int kReturnArmToLoadingButton = 4;
    public static final int kIntakeReleaseButton = 3;
    public static final int kReverseIntakeButton = 10;
    public static final int kReverseExteriorIntakeButton = 6;
    public static final int kTestButton = 11;

    // Zeroed values, should be in radians
    // source is google document in Electrical for team - module data
    public static final double kZeroedFrontLeft = 3.957;    // for FL module 5
    public static final double kZeroedFrontRight = 5.630;   // for FR module 4
    public static final double kZeroedBackLeft = 5.681;     // for BL module 2
    public static final double kZeroedBackRight = 1.815;    // for BR module 3
};