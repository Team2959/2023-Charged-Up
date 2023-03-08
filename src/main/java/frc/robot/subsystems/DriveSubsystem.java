// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class DriveSubsystem extends SubsystemBase {

    private SwerveModule m_frontLeft;
    private SwerveModule m_frontRight;
    private SwerveModule m_backLeft;
    private SwerveModule m_backRight;
    private AHRS m_navX;

    private boolean m_initalized = false;

    private SwerveDriveKinematics m_kinematics;
    private SwerveDriveOdometry m_odometry;

    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;// kMaxSpeedMetersPerSecond /
                                                                          // Math.hypot(0.381, 0.381);

    private static final double kHalfTrackWidthMeters = 0.571 / 2.0;
    private final Translation2d kFrontLeftLocation = new Translation2d(kHalfTrackWidthMeters, kHalfTrackWidthMeters);
    private final Translation2d kFrontRightLocation = new Translation2d(kHalfTrackWidthMeters, -kHalfTrackWidthMeters);
    private final Translation2d kBackLeftLocation = new Translation2d(-kHalfTrackWidthMeters, kHalfTrackWidthMeters);
    private final Translation2d kBackRightLocation = new Translation2d(-kHalfTrackWidthMeters, -kHalfTrackWidthMeters);
    private static final double kXBalancingP = 0.1;
    private static final double kXBalancingI = 0.0;
    private static final double kXBalancingD = 0.0;
    private static final double kYBalancingP = 0.1;
    private static final double kYBalancingI = 0.0;
    private static final double kYBalancingD = 0.0;

    final PIDController m_xBalancingController = new PIDController(kXBalancingP, kXBalancingI, kXBalancingD);
    final PIDController m_yBalancingController = new PIDController(kYBalancingP, kYBalancingI, kYBalancingD);

    private int m_ticks = 0;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem()
    {
        m_navX = new AHRS(I2C.Port.kMXP);

        m_kinematics = new SwerveDriveKinematics(kFrontLeftLocation, kFrontRightLocation, kBackLeftLocation,
                kBackRightLocation);

        m_frontLeft = new SwerveModule(RobotMap.kFrontLeftDriveCANSparkMaxMotor,
                RobotMap.kFrontLeftTurnCANSparkMaxMotor, RobotMap.kFrontLeftTurnPulseWidthDigIO,
                RobotMap.kZeroedFrontLeft, "Front Left");
        m_frontRight = new SwerveModule(RobotMap.kFrontRightDriveCANSparkMaxMotor,
                RobotMap.kFrontRightTurnCANSparkMaxMotor, RobotMap.kFrontRightTurnPulseWidthDigIO,
                RobotMap.kZeroedFrontRight, "Front Right");
        m_backLeft = new SwerveModule(RobotMap.kBackLeftDriveCANSparkMaxMotor,
                RobotMap.kBackLeftTurnCANSparkMaxMotor, RobotMap.kBackLeftTurnPulseWidthDigIO,
                RobotMap.kZeroedBackLeft, "Back Left");
        m_backRight = new SwerveModule(RobotMap.kBackRightDriveCANSparkMaxMotor,
                RobotMap.kBackRightTurnCANSparkMaxMotor, RobotMap.kBackRightTurnPulseWidthDigIO,
                RobotMap.kZeroedBackRight, "Back Right");

        m_odometry = new SwerveDriveOdometry(m_kinematics, getAngle(), getPositions());

        // m_frontLeft.driveSmartDashboardInit();
        // m_frontRight.driveSmartDashboardInit();
        // m_backLeft.driveSmartDashboardInit();
        // m_backRight.driveSmartDashboardInit();
    }

    public void initalize()
    {
        if (m_initalized)
            return;
        m_navX.reset();
        m_frontLeft.resetAngleEncoderToAbsolute();
        m_frontRight.resetAngleEncoderToAbsolute();
        m_backLeft.resetAngleEncoderToAbsolute();
        m_backRight.resetAngleEncoderToAbsolute();
        m_initalized = true;
    }

    public void offsetNavX(Rotation2d offset)
    {
        m_navX.setAngleAdjustment(offset.getDegrees());
    }

    @Override
    public void periodic()
    {
        m_odometry.update(getAngle(), getPositions());

        m_ticks++;
        if (m_ticks % 15 != 7)
            return;

        SmartDashboard.putNumber(getName() + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber(getName() + "/Roll", m_navX.getRoll());
        SmartDashboard.putNumber(getName() + "/Pitch", m_navX.getPitch());
    }

    public void smartDashboardInit() {
        m_frontLeft.driveSmartDashboardInit();
        m_frontRight.driveSmartDashboardInit();
        m_backLeft.driveSmartDashboardInit();
        m_backRight.driveSmartDashboardInit();
    }

    public void smartDashboardUpdate() {
        m_frontLeft.smartDashboardUpdate();
        m_frontRight.smartDashboardUpdate();
        m_backLeft.smartDashboardUpdate();
        m_backRight.smartDashboardUpdate();
    }

    public void drive(double xMetersPerSecond, double yMetersPerSecond,
            double rotationRadiansPerSecond, boolean fieldRelative)
    {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xMetersPerSecond, yMetersPerSecond, rotationRadiansPerSecond, getAngle())
                : new ChassisSpeeds(xMetersPerSecond, yMetersPerSecond, rotationRadiansPerSecond));

        SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeedMetersPerSecond);

        m_frontLeft.setDesiredState(states[0]);
        m_frontRight.setDesiredState(states[1]);
        m_backLeft.setDesiredState(states[2]);
        m_backRight.setDesiredState(states[3]);
    }

    public SwerveDriveKinematics getKinematics()
    {
        return m_kinematics;
    }

    public Pose2d getPose()
    {
        return m_odometry.getPoseMeters();
    }

    public void setDesiredState(SwerveModuleState[] states)
    {
        m_frontLeft.setDesiredState(states[0]);
        m_frontRight.setDesiredState(states[1]);
        m_backLeft.setDesiredState(states[2]);
        m_backRight.setDesiredState(states[3]);
    }

    public Rotation2d getAngle()
    {
        return m_navX.getRotation2d();
    }

    public void resetOdometry(Pose2d pose)
    {
        m_odometry.resetPosition(getAngle(), getPositions(), pose);
    }

    private SwerveModulePosition[] getPositions()
    {
        SwerveModulePosition[] swerveStates = { m_frontLeft.getPosition(), m_frontRight.getPosition(),
                m_backLeft.getPosition(), m_backRight.getPosition() };
        return swerveStates;
    }

}
