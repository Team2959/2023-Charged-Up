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
  public enum GamePieceType
  {
    Unknown,
    Cone,
    Cube
  };

  VictorSPX m_exteriorFeederMotors = new VictorSPX(RobotMap.kExteriorFeederVictorSpxMotor);
  Spark m_interiorFeederMotor = new Spark(RobotMap.kInteriorFeederSparkMotor);
  Spark m_flipperVacuumMotor = new Spark(RobotMap.kFlipperVacuumSparkMotor);
  SolenoidV2 m_intakeArm = new SolenoidV2(RobotMap.kIntakeArm);
  SolenoidV2 m_coneOrientor = new SolenoidV2(RobotMap.kConeOrientator);
  SolenoidV2 m_flipperArm = new SolenoidV2(RobotMap.kFlipperArm);
  SolenoidV2 m_intakeVacuumRelease = new SolenoidV2(RobotMap.kIntakeVacuumRelease);
  DigitalInput m_gamePieceDetected = new DigitalInput(RobotMap.kGamePieceDetectedSwitch);
  DigitalInput m_gamePieceIn = new DigitalInput(RobotMap.kGamePieceInSwitch);
  DigitalInput m_gamePieceIsCone = new DigitalInput(RobotMap.kConeAllInSwitch);
  double m_intakeSpeed = 1;
  double m_exteriorIntakeSpeed = 1.0;
  private GamePieceType m_GamePieceType = GamePieceType.Unknown;
  private int m_ticks = 0;

  @Override
  public void periodic() {
    m_ticks++;
    if (m_ticks % 15 != 11)
        return;

    intakeSmartDashboardUpdate();
  }

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem()
  {
    m_flipperArm.set(true);

    // intakeSmartDashboardInit();
  }

  public boolean gamePieceIsReady() {
    if(m_gamePieceIsCone.get()) {
      m_GamePieceType = GamePieceType.Cone;
      return true;
    }
    if(m_gamePieceIn.get()) {
        m_GamePieceType = GamePieceType.Cube;
        return true;
    }
    return false;
  }

  public boolean isVacuumEngaged() {
    return !m_intakeVacuumRelease.get();
  }

  public void toggleFlipper() {
    m_flipperArm.set(!m_flipperArm.get());
  }

  public void flipGamePiece() {
    m_flipperArm.set(true);
  }

  public void unflipGamePiece() {
    m_flipperArm.set(false);
  }

  public void startVacuum() {
    engageVacuumSeal();
    m_flipperVacuumMotor.set(1.0);
  }

  public void stopVacuum() {
    m_intakeVacuumRelease.set(true);
    m_flipperVacuumMotor.set(0.0);
  }

  public void engageVacuumSeal() {
    m_intakeVacuumRelease.set(false);
  }

  public boolean intakeIsDown() {
    return m_intakeArm.get();
  }

  public boolean gamePieceDetected() {
    return m_gamePieceDetected.get();
  }

  public void dropOrientatorBar() {
    m_coneOrientor.set(true);
  }

  public void pullUpOrientatorBar() {
    m_coneOrientor.set(false);
  }

  public void intakeSmartDashboardInit() {
    SmartDashboard.putNumber(getName() + "/Intake Speed", m_intakeSpeed);
    SmartDashboard.putNumber(getName() + "/Exterior Intake Speed", m_exteriorIntakeSpeed);
  }

  public void intakeSmartDashboardUpdate() {
    m_intakeSpeed = SmartDashboard.getNumber(getName() + "/Intake Speed", m_intakeSpeed);
    m_exteriorIntakeSpeed = SmartDashboard.getNumber(getName() + "/Exterior Intake Speed", m_exteriorIntakeSpeed);
    SmartDashboard.putBoolean(getName() + "/Game Piece Detected", m_gamePieceDetected.get());
    SmartDashboard.putBoolean(getName() + "/Game Piece In", m_gamePieceIn.get());
    SmartDashboard.putBoolean(getName() + "/Game Piece Is Cone", m_gamePieceIsCone.get());
    SmartDashboard.putNumber(getName() + "/PWM of Intake Belts", m_interiorFeederMotor.get());
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
    startVacuum();
    unflipGamePiece();
    m_coneOrientor.set(false);
  }

  public void turnOffIntake() {
    m_intakeArm.set(false);
    setExteriorFeederMotor(0);
    setInteriorFeederMotor(0);
    flipGamePiece();
    m_coneOrientor.set(false);
  }

  public void reverseAll(boolean exceptFlipper) {
    if(!exceptFlipper) {
      m_flipperArm.set(false);
    }

    m_coneOrientor.set(false);
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

  public GamePieceType getGamePieceType(){
    return m_GamePieceType;
  }
}
