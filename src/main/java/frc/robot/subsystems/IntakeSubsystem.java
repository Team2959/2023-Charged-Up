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

  public boolean seesColorYellow() {
    ColorMatchResult result = m_ColorMatcher.matchClosestColor(m_coneColorSensor.getColor());
    SmartDashboard.putNumber(getName() + "/Color Sensor/Confidence", result.confidence);
    return result.confidence > 0.9;
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
      m_intakeArm.set(false);
      m_exteriorFeederMotors.set(VictorSPXControlMode.PercentOutput, 0);
      m_interiorFeederMotor.set(0);
      m_coneOrientor.set(false);
    }
    else
    {
      //Droping the intake and activating the motors
      m_intakeArm.set(true);
      m_exteriorFeederMotors.set(VictorSPXControlMode.PercentOutput, 0.5);
      m_interiorFeederMotor.set(0.5);
      m_intakeVacuumRelease.set(true);
      m_flipperArm.set(true);
      m_coneOrientor.set(false);
    }
  }

  public GamePieceType getGamePieceType(){

    return m_GamePieceType;
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
  }
}
