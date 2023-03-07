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
  private VictorSPX m_exteriorFeederMotors = new VictorSPX(RobotMap.kExteriorFeederVictorSpxMotor);
  private Spark m_interiorFeederMotor = new Spark(RobotMap.kInteriorFeederSparkMotor);
  private Spark m_flipperVacuumMotor = new Spark(RobotMap.kFlipperVacuumSparkMotor);
  private CANSparkMax m_flipperMotor = new CANSparkMax(RobotMap.kFlipperSparkMaxMotor, MotorType.kBrushless);
  private SparkMaxPIDController m_flipperPIDController;
  private SparkMaxRelativeEncoder m_flipperEncoder;
  private SolenoidV2 m_intakeArm = new SolenoidV2(RobotMap.kIntakeArm);
  private SolenoidV2 m_coneOrienter = new SolenoidV2(RobotMap.kConeOrientater);
  private SolenoidV2 m_intakeVacuumRelease = new SolenoidV2(RobotMap.kIntakeVacuumRelease);
  private DigitalInput m_gamePieceDetected = new DigitalInput(RobotMap.kGamePieceDetectedSwitch);
  private DigitalInput m_gamePieceIn = new DigitalInput(RobotMap.kGamePieceInSwitch);
  private DigitalInput m_gamePieceIsUpright = new DigitalInput(RobotMap.kGamePieceUprightSwitch);
  private double m_intakeSpeed = 1;
  private double m_exteriorIntakeSpeed = 1.0;
 
  private static final double kDefaultFlipperDownPosition = -4;
  private double m_flippedPosition = kDefaultFlipperDownPosition;

  private static final double kFlipperP = 0.05;
  private static final double kFlipperI = 0;
  private static final double kFlipperD = 0;
  private static final double kFlipperFF = 0.005;
  private static final double kFlipperIZone = 0;

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

    flipGamePiece();
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

  public boolean isVacuumEngaged() {
    return !m_intakeVacuumRelease.get();
  }

  public void toggleFlipper() {
    if (m_flipperEncoder.getPosition() < -2)
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

  public void releaseVacuumSeal() {
    m_intakeVacuumRelease.set(true);
  }

  public void engageVacuumSeal() {
    m_intakeVacuumRelease.set(false);
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

    SmartDashboard.putNumber(getName() + "/Flipper Down Position", kDefaultFlipperDownPosition);
    SmartDashboard.putNumber(getName() + "/Current Flipper Position", m_flipperEncoder.getPosition());
    SmartDashboard.putNumber(getName() + "/Flipper P", m_flipperPIDController.getP());
    SmartDashboard.putNumber(getName() + "/Flipper I", m_flipperPIDController.getI());
    SmartDashboard.putNumber(getName() + "/Flipper D", m_flipperPIDController.getD());
    SmartDashboard.putNumber(getName() + "/Flipper IZone", m_flipperPIDController.getIZone());
    SmartDashboard.putNumber(getName() + "/Flipper FF", m_flipperPIDController.getFF());
  }

  public void smartDashboardUpdate() {
    m_intakeSpeed = SmartDashboard.getNumber(getName() + "/Intake Speed", m_intakeSpeed);
    m_exteriorIntakeSpeed = SmartDashboard.getNumber(getName() + "/Exterior Intake Speed", m_exteriorIntakeSpeed);
    SmartDashboard.putBoolean(getName() + "/Game Piece Detected", m_gamePieceDetected.get());
    SmartDashboard.putBoolean(getName() + "/Game Piece In", m_gamePieceIn.get());
    SmartDashboard.putBoolean(getName() + "/Game Piece Is Upright", m_gamePieceIsUpright.get());

    SmartDashboard.putNumber(getName() + "/Current Flipper Position", m_flipperEncoder.getPosition());
    m_flippedPosition = SmartDashboard.getNumber(getName() + "/Flipper Down Position", kDefaultFlipperDownPosition);
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
}
