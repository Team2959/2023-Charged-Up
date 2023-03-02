// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
  CANSparkMax m_flipperMotor = new CANSparkMax(RobotMap.kFlipperSparkMaxMotor, MotorType.kBrushless);
  private SparkMaxPIDController m_flipperPIDController;
  private SparkMaxRelativeEncoder m_flipperEncoder;
  SolenoidV2 m_intakeArm = new SolenoidV2(RobotMap.kIntakeArm);
  SolenoidV2 m_coneOrienter = new SolenoidV2(RobotMap.kConeOrientater);
  SolenoidV2 m_intakeVacuumRelease = new SolenoidV2(RobotMap.kIntakeVacuumRelease);
  DigitalInput m_gamePieceDetected = new DigitalInput(RobotMap.kGamePieceDetectedSwitch);
  DigitalInput m_gamePieceIn = new DigitalInput(RobotMap.kGamePieceInSwitch);
  DigitalInput m_gamePieceIsCone = new DigitalInput(RobotMap.kConeAllInSwitch);
  double m_intakeSpeed = 1;
  double m_exteriorIntakeSpeed = 1.0;
  private GamePieceType m_GamePieceType = GamePieceType.Unknown;
  private int m_ticks = 0;
  private double m_flippedPosition = 90;

  private static final double kFlipperP = 0.1;
  private static final double kFlipperI = 0;
  private static final double kFlipperD = 0;
  private static final double kFlipperFF = 0.01;
  private static final double kFlipperIZone = 0;

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
    m_flipperEncoder = (SparkMaxRelativeEncoder)m_flipperMotor.getEncoder();
    m_flipperPIDController = m_flipperMotor.getPIDController();
    m_flipperPIDController.setP(kFlipperP);
    m_flipperPIDController.setI(kFlipperI);
    m_flipperPIDController.setD(kFlipperD);
    m_flipperPIDController.setFF(kFlipperFF);
    m_flipperPIDController.setIZone(kFlipperIZone);

    intakeSmartDashboardInit();
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
    if (m_flipperEncoder.getPosition() > 10)
      flipGamePiece();
    else
      unflipGamePiece();
  }

  public void flipGamePiece() {
    m_flipperPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
  }

  public void unflipGamePiece() {
    m_flipperPIDController.setReference(m_flippedPosition, CANSparkMax.ControlType.kPosition);
  }

  public void startVacuum() {
    engageVacuumSeal();
    m_flipperVacuumMotor.set(1.0);
  }

  public void stopVacuum() {
    dropOrientatorBar();
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
    m_coneOrienter.set(true);
  }

  public void pullUpOrientatorBar() {
    m_coneOrienter.set(false);
  }

  public void intakeSmartDashboardInit() {
    SmartDashboard.putNumber(getName() + "/Intake Speed", m_intakeSpeed);
    SmartDashboard.putNumber(getName() + "/Exterior Intake Speed", m_exteriorIntakeSpeed);

    SmartDashboard.putNumber(getName() + "/Flipper Down Position", m_flippedPosition);
    SmartDashboard.putNumber(getName() + "/Flipper P", m_flipperPIDController.getP());
    SmartDashboard.putNumber(getName() + "/Flipper I", m_flipperPIDController.getI());
    SmartDashboard.putNumber(getName() + "/Flipper D", m_flipperPIDController.getD());
    SmartDashboard.putNumber(getName() + "/Flipper IZone", m_flipperPIDController.getIZone());
    SmartDashboard.putNumber(getName() + "/Flipper FF", m_flipperPIDController.getFF());
}

  public void intakeSmartDashboardUpdate() {
    m_intakeSpeed = SmartDashboard.getNumber(getName() + "/Intake Speed", m_intakeSpeed);
    m_exteriorIntakeSpeed = SmartDashboard.getNumber(getName() + "/Exterior Intake Speed", m_exteriorIntakeSpeed);
    SmartDashboard.putBoolean(getName() + "/Game Piece Detected", m_gamePieceDetected.get());
    SmartDashboard.putBoolean(getName() + "/Game Piece In", m_gamePieceIn.get());
    SmartDashboard.putBoolean(getName() + "/Game Piece Is Cone", m_gamePieceIsCone.get());
    SmartDashboard.putNumber(getName() + "/PWM of Intake Belts", m_interiorFeederMotor.get());

    m_flippedPosition = SmartDashboard.getNumber(getName() + "/Flipper Down Position", m_flippedPosition);
    m_flipperPIDController.setP(SmartDashboard.getNumber(getName() + "/Flipper P", kFlipperP));
    m_flipperPIDController.setI(SmartDashboard.getNumber(getName() + "/Flipper I", kFlipperI));
    m_flipperPIDController.setD(SmartDashboard.getNumber(getName() + "/Flipper D", kFlipperD));
    m_flipperPIDController.setIZone(SmartDashboard.getNumber(getName() + "/Flipper IZone", kFlipperIZone));
    m_flipperPIDController.setFF(SmartDashboard.getNumber(getName() + "/Flipper FF", kFlipperFF));
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
    m_coneOrienter.set(false);
  }

  public void turnOffIntake() {
    m_intakeArm.set(false);
    setExteriorFeederMotor(0);
    setInteriorFeederMotor(0);
    flipGamePiece();
    m_coneOrienter.set(false);
  }

  public void reverseAll(boolean exceptFlipper) {
    if(!exceptFlipper) {
      unflipGamePiece();
    }

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

  public GamePieceType getGamePieceType(){
    return m_GamePieceType;
  }
}
