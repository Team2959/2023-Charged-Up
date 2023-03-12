// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import cwtech.util.SolenoidV2;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IntakeSubsystem extends SubsystemBase
{
  private VictorSPX m_exteriorFeederMotors = new VictorSPX(RobotMap.kExteriorFeederVictorSpxMotor);
  private Spark m_interiorFeederMotor = new Spark(RobotMap.kInteriorFeederSparkMotor);
  private SolenoidV2 m_intakeArm = new SolenoidV2(RobotMap.kIntakeArm);
  private SolenoidV2 m_coneOrienter = new SolenoidV2(RobotMap.kConeOrientater);
  private DigitalInput m_gamePieceDetected = new DigitalInput(RobotMap.kGamePieceDetectedSwitch);
  private DigitalInput m_gamePieceIn = new DigitalInput(RobotMap.kGamePieceInSwitch);
  private DigitalInput m_gamePieceIsUpright = new DigitalInput(RobotMap.kGamePieceUprightSwitch);
  private double m_intakeSpeed = 1;
  private double m_exteriorIntakeSpeed = 1.0;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
  }

  public boolean gamePieceDetected() {
    return !m_gamePieceDetected.get();
  }

  public boolean gamePieceIsReadyToFlip() {
    return !m_gamePieceIn.get();
  }

  public boolean gamePieceIsReadyToLoad() {
    return !m_gamePieceIsUpright.get();
  }

  public boolean intakeIsDown() {
    return m_intakeArm.get();
  }

  public void dropOrientatorBar() {
    m_coneOrienter.set(true);
  }

  public void pullUpOrientatorBar() {
    m_coneOrienter.set(false);
  }

  public void smartDashboardInit() {
    SmartDashboard.putNumber(getName() + "/Intake Speed", m_intakeSpeed);
    SmartDashboard.putNumber(getName() + "/Exterior Intake Speed", m_exteriorIntakeSpeed);
  }

  public void smartDashboardUpdate() {
    m_intakeSpeed = SmartDashboard.getNumber(getName() + "/Intake Speed", m_intakeSpeed);
    m_exteriorIntakeSpeed = SmartDashboard.getNumber(getName() + "/Exterior Intake Speed", m_exteriorIntakeSpeed);
    SmartDashboard.putBoolean(getName() + "/Game Piece Detected", m_gamePieceDetected.get());
    SmartDashboard.putBoolean(getName() + "/Game Piece In", m_gamePieceIn.get());
    SmartDashboard.putBoolean(getName() + "/Game Piece Is Upright", m_gamePieceIsUpright.get());
  }

  public void toggleIntakeSubsystem()
  {
    if (m_intakeArm.get())
    {
      //retracting the intake and deactivating the motors
      turnOffIntake();
    }
    else
    {
      //Droping the intake and activating the motors
      turnOnIntake();
    }
  }

  public void setExteriorFeederMotor(double output) {
    m_exteriorFeederMotors.set(VictorSPXControlMode.PercentOutput, output);
  }

  public void setInteriorFeederMotor(double output) {
    m_interiorFeederMotor.set(output);
  }

  public void turnOnIntake() {
    m_intakeArm.set(true);
    setExteriorFeederMotor(m_exteriorIntakeSpeed);
    setInteriorFeederMotor(m_intakeSpeed);
    m_coneOrienter.set(false);
  }

  public void turnOffIntake() {
    m_intakeArm.set(false);
    setExteriorFeederMotor(0);
    setInteriorFeederMotor(0);
    m_coneOrienter.set(false);
  }

  public void reverseAll() {
    m_coneOrienter.set(false);
    setExteriorFeederMotor(-m_exteriorIntakeSpeed);
    setInteriorFeederMotor(-m_intakeSpeed);
  }

  public void reverseFront() {
    setExteriorFeederMotor(-m_exteriorIntakeSpeed);
  }

  public void stopIntake() {
    setExteriorFeederMotor(0);
    setInteriorFeederMotor(0);
  }
}
