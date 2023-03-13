// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ArmRotationSubsystem extends SubsystemBase {
  public static final double kArmHomePosition = 60;
  private static final double kArmRotatorP = 0.0175;
  private static final double kArmRotatorI = 0;
  private static final double kArmRotatorD = 0;
  private static final double kArmRotatorMaxVelocity = 50; // degrees per second
  private static final double kArmRotatorMaxAccel = 25; // degrees per second per second
  private double m_lastArmRotationTarget = kArmHomePosition;

  private CANSparkMax m_armRotatorMotor = new CANSparkMax(RobotMap.kArmRotatorSparkMaxMotor, MotorType.kBrushless);
  private DigitalInput m_armRotatorEncoderDigitalInput = new DigitalInput(RobotMap.kRotatorArmEncoderPulseWidthDIO);
  private DutyCycle m_armRotatorEncoderDutyCycle = new DutyCycle(m_armRotatorEncoderDigitalInput);
  // current, should be deleted when ProfilePID works
  private static double kRotationLimitDivisor = 3.0;
  private static double kRotationLimitContinue = 0.25;
  private PIDController m_armRotatorMotorPidController =
    new PIDController(kArmRotatorP, kArmRotatorI, kArmRotatorD);
  // to replcae PIDController for rotation
  private ProfiledPIDController m_armRotatorMotorProfiledPidController =
    new ProfiledPIDController(kArmRotatorP, kArmRotatorI, kArmRotatorD,
        new Constraints(kArmRotatorMaxVelocity, kArmRotatorMaxAccel));

  /** Creates a new ArmRotationSubsystem. */
  public ArmRotationSubsystem() {
    m_armRotatorMotor.setIdleMode(IdleMode.kBrake);

    setArmDegrees(kArmHomePosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(getName() + "/Arm Rotation Encoder Position", getArmAngle());

        // TODO try profiled
        double rawProfiled = m_armRotatorMotorProfiledPidController.calculate(getArmAngle(), m_lastArmRotationTarget);
        // m_armRotatorMotor.set(rawProfiled);
        SmartDashboard.putNumber(getName() + "/Arm Rotator Profiled Raw Output", rawProfiled);

        double raw =  m_armRotatorMotorPidController.calculate(getArmAngle());
        var diff = Math.abs(raw);
        if (diff > kRotationLimitContinue)
            raw /= kRotationLimitDivisor;
        SmartDashboard.putNumber(getName() + "/Arm Rotator Raw Output", raw);
        m_armRotatorMotor.set(raw);
  }

  public void setArmDegrees(double degrees) {
    // simple arm PID control
    m_armRotatorMotorPidController.setSetpoint(degrees);
    // profiled PID control
    // m_armRotatorMotorProfiledPidController.setGoal(degrees);
    m_lastArmRotationTarget = degrees;
  }

  public void stopArmRotatorMotor() {
    setArmDegrees(getArmAngle());
  }

  public boolean isArmRotatorAtSetpoint() {
    return Math.abs(m_lastArmRotationTarget - getArmAngle()) < 5;
  }

  public double getArmAngle() {
    return m_armRotatorEncoderDutyCycle.getOutput() * 360;
  }

  public void smartDashboardInit() {
    SmartDashboard.putBoolean(getName() + "/Arm Rot Prof Const Enable", false);
    SmartDashboard.putNumber(getName() + "/Arm Rotator Max Accel", kArmRotatorMaxAccel);
    SmartDashboard.putNumber(getName() + "/Arm Rotator Max Vel", kArmRotatorMaxVelocity);

    SmartDashboard.putNumber(getName() + "/Arm Rotator P", m_armRotatorMotorPidController.getP());
    SmartDashboard.putNumber(getName() + "/Arm Rotator I", m_armRotatorMotorPidController.getI());
    SmartDashboard.putNumber(getName() + "/Arm Rotator D", m_armRotatorMotorPidController.getD());
}

public void smartDashboardUpdate() {
    if (SmartDashboard.getBoolean(getName() + "/Arm Rot Prof Const Enable", false))
    {
        var maxAccelRot = SmartDashboard.getNumber(getName() + "/Arm Rotator Max Accel", kArmRotatorMaxAccel);
        var maxVelRot = SmartDashboard.getNumber(getName() + "/Arm Rotator Max Vel", kArmRotatorMaxVelocity);
        m_armRotatorMotorProfiledPidController.setConstraints(new Constraints(maxVelRot, maxAccelRot));
    }

    var rotP = SmartDashboard.getNumber(getName() + "/Arm Rotator P", kArmRotatorP);
    var rotI = SmartDashboard.getNumber(getName() + "/Arm Rotator I", kArmRotatorI);
    var rotD = SmartDashboard.getNumber(getName() + "/Arm Rotator D", kArmRotatorD);

    m_armRotatorMotorPidController.setPID(rotP, rotI, rotD);
    m_armRotatorMotorProfiledPidController.setPID(rotP, rotI, rotD);
  }
}
