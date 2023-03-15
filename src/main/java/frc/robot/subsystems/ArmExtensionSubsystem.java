// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ArmExtensionSubsystem extends SubsystemBase {
    private static final int kSmartMaxVel = 7500;
    private static final int kSmartMaxAccel = 10000;
    private static final double kArmExtensionP = 0.02;  // orig kP from St Joe: 0.02
    private static final double kArmExtensionI = 0;  // orig KI from St Joe: 0
    // private static final double kArmExtensionP = 0.000075;
    // private static final double kArmExtensionI = 0.000001;
    private static final double kArmExtensionD = 0;
    private static final double kArmExtensionFF = 0;
    private static final double kArmExtensionIzone = 0;

    private CANSparkMax m_armExtensionMotor = new CANSparkMax(RobotMap.kArmExtensionSparkMaxMotor,
            MotorType.kBrushless);
    // RopeSensor m_ropeSensor = new RopeSensor(RobotMap.kRopeEncoderDigIO);
    private SparkMaxRelativeEncoder m_extensionEncoder;
    private SparkMaxPIDController m_armExtensionMotorPidController;
    private double m_lastArmExtensionTarget = 0; // Arm starts fully retracted

    // private ProfiledPIDController m_armExtensionMotorProfiledPidController =
    // new ProfiledPIDController(kArmExtensionP, kArmExtensionI, kArmExtensionD,
    //     new Constraints(kArmExtensionMaxVelocity, kArmExtensionMaxAccel));

    /** Creates a new ArmExtensionSubsystem. */
    public ArmExtensionSubsystem() {
        m_armExtensionMotor.restoreFactoryDefaults();
        m_armExtensionMotor.setIdleMode(IdleMode.kBrake);

        m_extensionEncoder = (SparkMaxRelativeEncoder) m_armExtensionMotor.getEncoder();
        m_armExtensionMotorPidController = m_armExtensionMotor.getPIDController();

        m_armExtensionMotorPidController.setP(kArmExtensionP);
        m_armExtensionMotorPidController.setI(kArmExtensionI);
        m_armExtensionMotorPidController.setD(kArmExtensionD);
        m_armExtensionMotorPidController.setFF(kArmExtensionFF);
        m_armExtensionMotorPidController.setIZone(kArmExtensionIzone);
        // https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Smart%20Motion%20Example/src/main/java/frc/robot/Robot.java
        // m_armExtensionMotorPidController.setSmartMotionMaxAccel(kSmartMaxAccel, 0);
        // m_armExtensionMotorPidController.setSmartMotionMaxVelocity(kSmartMaxVel, 0);

        // m_extensionEncoder.setPosition(0);
        // setArmExtensionPosition(getArmExtensionPosition());
    }

    // public void setupRopeSensor(Robot robot) {
    // m_ropeSensor.setup(robot);
    // }

    private static int kConeExtensionAutoStartPosition = 57;

    public void onAutoInit() {
        // ToDo: handle diffferent autos, cube as well as cone
        // if(m_quickFixCone) {
        m_extensionEncoder.setPosition(kConeExtensionAutoStartPosition);
        setArmExtensionPosition(kConeExtensionAutoStartPosition);
        m_armExtensionMotor.set(0);
        // }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber(getName() + "/Arm Extension Ticks", getArmExtensionPosition());

        // double rawProfiled = m_armExtensionMotorProfiledPidController.calculate(getArmExtensionPosition());
        // m_armExtensionMotor.set(rawProfiled);
        // SmartDashboard.putNumber(getName() + "/Arm Extension Profiled Raw Output", rawProfiled);

        SmartDashboard.putNumber(getName() + "/Arm Extension Raw Output", m_armExtensionMotor.get());
        SmartDashboard.putNumber(getName() + "/Arm Extension Applied Output", m_armExtensionMotor.getAppliedOutput());
        SmartDashboard.putNumber(getName() + "/Arm Extension Output Current", m_armExtensionMotor.getOutputCurrent());

        // If switch back to rope encoder
        // m_armExtensionMotor.set(m_armExtensionMotorPidController.calculate(getArmExtensionPosition()));
    }

    public void setArmExtensionPosition(double distance) {
        // m_armExtensionMotor.set(0); // stop motor so us modifying the going forward
        // doesn't screw things up

        // rope sensor control
        // m_ropeSensor.setGoingForward(distance > m_lastArmExtensionTarget);
        // m_armExtensionMotorPidController.setSetpoint(distance);

        // Regular position control
        m_armExtensionMotorPidController.setReference(distance, CANSparkMax.ControlType.kPosition);

        // Smart Motion position control
        // m_armExtensionMotorPidController.setReference(distance, CANSparkMax.ControlType.kSmartMotion);

        m_lastArmExtensionTarget = distance;
    }

    public void stopArmExtensionMotor() {
        setArmExtensionPosition(getArmExtensionPosition());
    }

    public boolean isArmExtensionAtSetpoint() {
        return Math.abs(m_lastArmExtensionTarget - getArmExtensionPosition()) < 5;
    }

    public double getArmExtensionPosition() {
        // return m_ropeSensor.getTicks();
        return m_extensionEncoder.getPosition();
    }

    public void smartDashboardInit() {
        SmartDashboard.putBoolean(getName() + "/Arm Extension Prof Const Enable", false);

        SmartDashboard.putNumber(getName() + "/Arm Extension P", m_armExtensionMotorPidController.getP());
        SmartDashboard.putNumber(getName() + "/Arm Extension I", m_armExtensionMotorPidController.getI());
        SmartDashboard.putNumber(getName() + "/Arm Extension D", m_armExtensionMotorPidController.getD());
        SmartDashboard.putNumber(getName() + "/Arm Extension FF", m_armExtensionMotorPidController.getFF());
        SmartDashboard.putNumber(getName() + "/Arm Extension Izone", m_armExtensionMotorPidController.getIZone());

        SmartDashboard.putNumber(getName() + "/Arm Extension Max Accel", kSmartMaxAccel);
        SmartDashboard.putNumber(getName() + "/Arm Extension Max Vel", kSmartMaxVel);
    }

    public void smartDashboardUpdate() {
        m_armExtensionMotorPidController.setP(SmartDashboard.getNumber(getName() + "/Arm Extension P", kArmExtensionP));
        m_armExtensionMotorPidController.setI(SmartDashboard.getNumber(getName() + "/Arm Extension I", kArmExtensionI));
        m_armExtensionMotorPidController.setD(SmartDashboard.getNumber(getName() + "/Arm Extension D", kArmExtensionD));
        m_armExtensionMotorPidController.setFF(SmartDashboard.getNumber(getName() + "/Arm Extension FF", kArmExtensionFF));
        m_armExtensionMotorPidController.setIZone(SmartDashboard.getNumber(getName() + "/Arm Extension Izone", kArmExtensionIzone));

        if (SmartDashboard.getBoolean(getName() + "/Arm Extension Prof Const Enable", false))
        {
            var maxAccelExtension = SmartDashboard.getNumber(getName() + "/Arm Rotator Max Accel", kSmartMaxAccel);
            var maxVelExtension = SmartDashboard.getNumber(getName() + "/Arm Rotator Max Vel", kSmartMaxVel);
            // m_armExtensionMotorPidController.setSmartMotionMaxAccel(maxAccelExtension, 0);
            // m_armExtensionMotorPidController.setSmartMotionMaxVelocity(maxVelExtension, 0);
            // m_armExtensionMotorProfiledPidController.setConstraints(new Constraints(maxVelRot, maxAccelRot));
        }

        SmartDashboard.putNumber(getName() + "/Arm Extension Velocity", m_extensionEncoder.getVelocity());
        SmartDashboard.putNumber(getName() + "/Arm Extension Position", m_extensionEncoder.getPosition());
    }
}
