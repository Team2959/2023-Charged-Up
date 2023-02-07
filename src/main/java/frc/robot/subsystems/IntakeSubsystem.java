// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IntakeSubsystem extends SubsystemBase
{

  VictorSPX m_exteriorFeederMotors = new VictorSPX(RobotMap.kExteriorFeederVictorSpxMotor);
  Spark m_interiorFeederMotor = new Spark(RobotMap.kInteriorFeederSparkMotor);
  Spark m_flipperVacuumMotor = new Spark(RobotMap.kFlipperVacuumSparkMotor);
  Solenoid m_intakeArm = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.kIntakeArm);
  Solenoid m_coneOrientor = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.kConeOrientator);
  Solenoid m_flipperArm = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.kFlipperArm);
  Solenoid m_intakeVacuumRelease = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.kIntakeVacuumRelease);
  DigitalInput m_gamePeicePresent = new DigitalInput(RobotMap.kGamePeicePresentSwitch);
  DigitalInput m_gamePeiceAllIn = new DigitalInput(RobotMap.kGamePeiceAllInSwitch);
  ColorSensorV3 m_coneColorSensor = new ColorSensorV3(I2C.Port.kOnboard);

  
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
   
  }


  public void toggleIntakeSubsystem() {
    if (m_intakeArm.get()) {
      //retracting the intake and deactivating the motors
      m_intakeArm.set(false);
      m_exteriorFeederMotors.set(VictorSPXControlMode.PercentOutput, 0);
      m_interiorFeederMotor.set(0);
      m_coneOrientor.set(false);

    }
    else {
      //Droping the intake and activating the motors
      m_intakeArm.set(true);
      m_exteriorFeederMotors.set(VictorSPXControlMode.PercentOutput, 0.5);
      m_interiorFeederMotor.set(0.5);
      m_intakeVacuumRelease.set(true);
      m_flipperArm.set(true);
      m_coneOrientor.set(false);

    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
