// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
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

  private GamePieceType m_GamePieceType = GamePieceType.Unknown;

  VictorSPX m_exteriorFeederMotors = new VictorSPX(RobotMap.kExteriorFeederVictorSpxMotor);
  Spark m_interiorFeederMotor = new Spark(RobotMap.kInteriorFeederSparkMotor);
  Spark m_flipperVacuumMotor = new Spark(RobotMap.kFlipperVacuumSparkMotor);
  Solenoid m_intakeArm = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.kIntakeArm);
  Solenoid m_coneOrientor = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.kConeOrientator);
  Solenoid m_flipperArm = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.kFlipperArm);
  Solenoid m_intakeVacuumRelease = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.kIntakeVacuumRelease);
  DigitalInput m_gamePiecePresent = new DigitalInput(RobotMap.kGamePeicePresentSwitch);
  DigitalInput m_gamePieceAllIn = new DigitalInput(RobotMap.kConeAllInSwitch);
  ColorSensorV3 m_coneColorSensor = new ColorSensorV3(I2C.Port.kMXP);
  ColorMatch m_ColorMatcher = new ColorMatch();
  double m_intakeSpeed = 0.5;


  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem()
  {
    m_ColorMatcher.addColorMatch(new Color(1.0, 1.0, 0.0));
  }

  public boolean gamePieceIsReady() {
    if(m_gamePieceAllIn.get()) {
      return true;
    }
    else if(m_gamePiecePresent.get()) {
      if(!seesColorYellow()) {
        return true;
      }
    }
    return false;
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
    m_intakeVacuumRelease.set(false);
    m_flipperVacuumMotor.set(1.0);
  }

  public void stopVacuum() {
    m_intakeVacuumRelease.set(true);
    m_flipperVacuumMotor.set(0.0);
  }

  public boolean seesColorYellow() {
    ColorMatchResult result = m_ColorMatcher.matchClosestColor(m_coneColorSensor.getColor());
    SmartDashboard.putNumber(getName() + "/Color Sensor/Confidence", result.confidence);
    return result.confidence > 0.9;
  }

  public boolean intakeIsDown() {
    return m_intakeArm.get();
  }

  public void dropOrientatorBar() {
    m_coneOrientor.set(true);
  }

  public void pullUpOrientatorBar() {
    m_coneOrientor.set(false);
  }

  public void intakeSmartDashboardInit() {
    SmartDashboard.putNumber(getName() + "/Intake Speed", m_intakeSpeed);
  }

  public void intakeSmartDashboardUpdate() {
    m_intakeSpeed = SmartDashboard.getNumber(getName() + "/Intake Speed", m_intakeSpeed);
    SmartDashboard.putBoolean(getName() + "/Game Piece Present", m_gamePiecePresent.get());
    SmartDashboard.putBoolean(getName() + "/Game Piece All In", m_gamePieceAllIn.get());
    SmartDashboard.putNumber(getName() + "/Color Sensor/R", m_coneColorSensor.getRed());
    SmartDashboard.putNumber(getName() + "/Color Sensor/G", m_coneColorSensor.getGreen());
    SmartDashboard.putNumber(getName() + "/Color Sensor/B", m_coneColorSensor.getBlue());
    SmartDashboard.putBoolean(getName() + "/Color Sensor Sees Yellow", seesColorYellow());
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
    setExteriorFeederMotor(0.5);
    setInteriorFeederMotor(0.5);
    m_intakeVacuumRelease.set(true);
    m_flipperArm.set(true);
    m_coneOrientor.set(false);
  }

  public void turnOffIntake() {
    m_intakeArm.set(false);
    setExteriorFeederMotor(0);
    setInteriorFeederMotor(0);
    m_coneOrientor.set(false);
  }

  public void reverseAll(boolean exceptFlipper) {
    if(!exceptFlipper) {
      m_flipperArm.set(false);
    }

    m_coneOrientor.set(false);
    setExteriorFeederMotor(-0.5);
    setInteriorFeederMotor(-0.5);
  }

  public void reverseFront() {
    setExteriorFeederMotor(-0.5);
  }

  public void stopIntake() {
    setExteriorFeederMotor(0);
    setInteriorFeederMotor(0);
  }

  public GamePieceType getGamePieceType(){

    return m_GamePieceType;
  }
}
