// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class DriveSubsystem extends SubsystemBase {

  private SwerveModule m_frontLeft;
  private SwerveModule m_frontRight;
  private SwerveModule m_backLeft;
  private SwerveModule m_backRight;
  private AHRS m_navX;

  private SwerveDriveKinematics m_kinematics;
  private SwerveDriveOdometry m_odometry;

  public static final double kMaxSpeedMetersPerSecond = 4;
  public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;// kMaxSpeedMetersPerSecond / Math.hypot(0.381, 0.381);
  
  // private final Translation2d kFrontLeftLocation = new Translation2d(0.381, 0.381);
  // private final Translation2d kFrontRightLocation = new Translation2d(0.381, -0.381);
  // private final Translation2d kBackLeftLocation = new Translation2d(-0.381, 0.381);
  // private final Translation2d kBackRightLocation = new Translation2d(-0.381, -0.381);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    m_navX = new AHRS(Port.kMXP);
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
  }

  @Override
  public void periodic() {
    SwerveModulePosition[] swerveStates = {m_frontLeft.getPosition(), m_frontRight.getPosition(),
      m_backLeft.getPosition(), m_backRight.getPosition()};
    m_odometry.update(m_navX.getRotation2d(), swerveStates);
  }

  public void drive(double xMetersPerSecond, double yMetersPerSecond,
      double rotationRadiansPerSecond, boolean fieldRelative)
  {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xMetersPerSecond, yMetersPerSecond, rotationRadiansPerSecond,
            m_navX.getRotation2d())
        : new ChassisSpeeds(xMetersPerSecond, yMetersPerSecond, rotationRadiansPerSecond));

    SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(states[0]);
    m_frontRight.setDesiredState(states[1]);
    m_backLeft.setDesiredState(states[2]);
    m_backRight.setDesiredState(states[3]);
  }

  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void setDesiredState(SwerveModuleState[] states) {
    m_frontLeft.setDesiredState(states[0]);
    m_frontRight.setDesiredState(states[1]);
    m_backLeft.setDesiredState(states[2]);
    m_backRight.setDesiredState(states[3]);
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(m_navX.getRotation2d(), getPositions(), pose);
  }
  private SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] swerveStates = {m_frontLeft.getPosition(), m_frontRight.getPosition(),
      m_backLeft.getPosition(), m_backRight.getPosition()};
      return swerveStates;
  }
}
