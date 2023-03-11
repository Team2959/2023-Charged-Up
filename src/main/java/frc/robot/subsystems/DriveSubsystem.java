// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.Vision.BotPose;

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
    private static final double kXBalancingP = 0.028;
    private static final double kXBalancingI = 0.006;
    private static final double kXBalancingD = 0.0;
    private static final double kYBalancingP = 0.028;
    private static final double kYBalancingI = 0.006;
    private static final double kYBalancingD = 0.0;
    private static final double kRotationP = 0.1;
    private static final double kRotationI = 0.0;
    private static final double kRotationD = 0.0;

    final ProfiledPIDController m_xBalancingController = new ProfiledPIDController(kXBalancingP, kXBalancingI, kXBalancingD, new Constraints(0.2, 0.2));
    final ProfiledPIDController m_yBalancingController = new ProfiledPIDController(kYBalancingP, kYBalancingI, kYBalancingD, new Constraints(0.2, 0.2));
    public boolean m_balancing = false;
    boolean m_invertRollAndPitch = false;
    boolean m_invertXInput = false;
    boolean m_invertYInput = false;

    final PIDController m_rotationController = new PIDController(kRotationP, kRotationI, kRotationD);
    double m_angleToRotateTo = 0.0;
    boolean m_rotationAlignmentOn = false;
    
    double autoRotateValue = 0.0;
    double autoXValue = 0.0;
    double autoYValue = 0.0;

    private int m_ticks = 0;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
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
    }

    public void initalize() {
        if (m_initalized)
            return;
        m_navX.reset();
        m_frontLeft.resetAngleEncoderToAbsolute();
        m_frontRight.resetAngleEncoderToAbsolute();
        m_backLeft.resetAngleEncoderToAbsolute();
        m_backRight.resetAngleEncoderToAbsolute();
        m_initalized = true;
    }

    public void offsetNavX(Rotation2d offset) {
        m_navX.setAngleAdjustment(offset.getDegrees());
    }

    public void turnOnBalancing() {
        m_balancing = true;
        autoBalanceDirection();
        m_xBalancingController.setGoal(0);
        m_yBalancingController.setGoal(0);
    }

    private boolean m_autoBalanceStartAngle = true;
    private int m_autoBalanceDelayTicks = 0;
    private int m_autoBalanceDelayTicksMax = 5;
    private double m_autoBalanceStopAngle = 12.5;
    private void autoBalanceDirection()
    {
        m_autoBalanceDelayTicks = 0;
        m_autoBalanceStartAngle = m_navX.getPitch() > 0;
    }

    public void turnOffBalancing() {
        m_balancing = false;
    }

    public void turnOnRotate() {
        double diff180 = Math.abs(180 - getAngle().getDegrees());
        double diff0 = Math.abs(getAngle().getDegrees());
        if(diff0 > diff180) {
            m_angleToRotateTo = 180;
        } else {
            m_angleToRotateTo = 0;
        }
        m_rotationAlignmentOn = true;
    }

    public void turnOffRotate() {
        m_rotationAlignmentOn = false;
    }

    @Override
    public void periodic() {
        m_odometry.update(getAngle(), getPositions());
        
        // TODO try this code
        if (m_balancing) {
            // TODO trapezoidal
            double xInput = 0.0;
            double yInput = 0.0;
            if(m_invertRollAndPitch) {
                xInput = m_navX.getRoll();
                yInput = m_navX.getPitch();
            }
            else {
                xInput = m_navX.getPitch();
                yInput = m_navX.getRoll();
            }

            if(m_invertXInput) {
                xInput *= -1;
            } 
            if(m_invertYInput) {
                yInput *= -1;
            }

            if (m_autoBalanceDelayTicks > 0 )
            {
                m_autoBalanceDelayTicks++;
                if (m_autoBalanceDelayTicks < m_autoBalanceDelayTicksMax)
                    return;

                autoBalanceDirection();
            }

            if ((m_autoBalanceStartAngle && m_navX.getPitch() < m_autoBalanceStopAngle) ||
                (!m_autoBalanceStartAngle && m_navX.getPitch() > -m_autoBalanceStopAngle))
            {
                m_autoBalanceDelayTicks = 1;
                drive(0, 0, 0, true);

                return;
            }

            autoXValue = -m_xBalancingController.calculate(xInput);
            // autoYValue = m_yBalancingController.calculate(yInput);
            SmartDashboard.putNumber(getName() + "/Balance Auto X", autoXValue);
            SmartDashboard.putNumber(getName() + "/Balance Auto Y", autoYValue);

            drive(autoXValue, 0, 0, true); // TODO check we want field centric
        }

        // if(m_rotationAlignmentOn) {
        //     // TODO trapezoidal
        //     autoRotateValue = m_rotationController.calculate(m_navX.getAngle());
        // }

        m_ticks++;
        if (m_ticks % 15 != 7)
            return;

        SmartDashboard.putNumber(getName() + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber(getName() + "/Roll", m_navX.getRoll());
        SmartDashboard.putNumber(getName() + "/Pitch", m_navX.getPitch());
        
        SmartDashboard.putData(m_yBalancingController);
        // BotPose botpose = Vision.getBotPose();
        // SmartDashboard.putNumber(getName() + "/Distance X", botpose.getX());
        // SmartDashboard.putNumber(getName() + "/Distance Y", botpose.getY());
        // SmartDashboard.putNumber(getName() + "/Distance Z", botpose.getZ());
    }

    public void smartDashboardInit() {
        m_frontLeft.driveSmartDashboardInit();
        m_frontRight.driveSmartDashboardInit();
        m_backLeft.driveSmartDashboardInit();
        m_backRight.driveSmartDashboardInit();
        SmartDashboard.putBoolean(getName() + "/Invert Roll and Pitch", m_invertRollAndPitch);
        SmartDashboard.putBoolean(getName() + "/Invert X", m_invertXInput);
        SmartDashboard.putBoolean(getName() + "/Invert Y", m_invertYInput);
        SmartDashboard.putData(getName() + "/X Balancing PID Controller", m_xBalancingController);
        SmartDashboard.putData(getAngle() + "/Y Balancing PID Controller", m_yBalancingController);
        SmartDashboard.putNumber(getName() + "/Balance Auto Delay Ticks", 5);
        SmartDashboard.putNumber(getName() + "/Balance Auto Stop Angle", 5);
    }

    public void smartDashboardUpdate() {
        m_invertRollAndPitch = SmartDashboard.getBoolean(getName() + "/Invert Roll and Pitch", m_invertRollAndPitch);
        m_invertXInput = SmartDashboard.getBoolean(getName() + "/Invert X", m_invertXInput);
        m_invertYInput = SmartDashboard.getBoolean(getName() + "/Invert Y", m_invertYInput);
        m_autoBalanceDelayTicksMax = (int)SmartDashboard.getNumber(getName() + "/Balance Auto Delay Ticks", 5);
        m_autoBalanceStopAngle = SmartDashboard.getNumber(getName() + "/Balance Auto Stop Angle", 5);

        m_frontLeft.smartDashboardUpdate();
        m_frontRight.smartDashboardUpdate();
        m_backLeft.smartDashboardUpdate();
        m_backRight.smartDashboardUpdate();
    }

    public void drive(double xMetersPerSecond, double yMetersPerSecond,
            double rotationRadiansPerSecond, boolean fieldRelative) {
        // if(m_balancing) {
        //     xMetersPerSecond = autoXValue * DriveSubsystem.kMaxSpeedMetersPerSecond;
        //     yMetersPerSecond = autoYValue * DriveSubsystem.kMaxSpeedMetersPerSecond;
        // }
        // if(m_rotationAlignmentOn) {
        //     rotationRadiansPerSecond = Math.toRadians(autoRotateValue * 360);
        // }


        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xMetersPerSecond, yMetersPerSecond, rotationRadiansPerSecond,
                        getAngle())
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

    public Rotation2d getAngle() {
        return m_navX.getRotation2d();
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(getAngle(), getPositions(), pose);
    }

    private SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] swerveStates = { m_frontLeft.getPosition(), m_frontRight.getPosition(),
                m_backLeft.getPosition(), m_backRight.getPosition() };
        return swerveStates;
    }

    public void turn180() {
        m_frontLeft.setAngle(90);
        m_frontRight.setAngle(90);
        m_backLeft.setAngle(90);
        m_backRight.setAngle(90);
    }

}
