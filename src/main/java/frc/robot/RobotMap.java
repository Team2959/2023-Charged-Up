// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class RobotMap {

    // CAN motor addresses
    public static final int kFrontLeftDriveCANSparkMaxMotor = 1;
    public static final int kFrontRightDriveCANSparkMaxMotor = 2;
    public static final int kBackRightDriveCANSparkMaxMotor = 3;
    public static final int kBackLeftDriveCANSparkMaxMotor = 4;
    public static final int kFrontLeftTurnCANSparkMaxMotor = 11;
    public static final int kFrontRightTurnCANSparkMaxMotor = 12;
    public static final int kBackRightTurnCANSparkMaxMotor = 13;
    public static final int kBackLeftTurnCANSparkMaxMotor = 14;

    // PWM motor addresses
    // Servos

    // REV Pneumatic Hub solenoid addresses

    // String Pot

    // Digital IO addresses
    public static final int kFrontLeftTurnPulseWidthDigIO = 1;
    public static final int kFrontRightTurnPulseWidthDigIO = 2;
    public static final int kBackRightTurnPulseWidthDigIO = 3;
    public static final int kBackLeftTurnPulseWidthDigIO = 4;

    // Operator input USB ports
    public static final int kLeftJoystick = 0;
    public static final int kRightJoystick = 1;
    public static final int kButtonBox = 2;

    // Co-Pilot Buttons
    public static final int kFireButton = 1;

    // Driver Buttons
    public static final int kToggleIntakeButton = 2;

    // Zeroed values, should be in radians
    public static final double kZeroedFrontLeft = 3.082 + Math.toRadians(90);
    public static final double kZeroedFrontRight = 0 + Math.toRadians(90);
    public static final double kZeroedBackLeft = 0.520 + Math.toRadians(90);
    public static final double kZeroedBackRight = -0.230 + Math.toRadians(90);
}
