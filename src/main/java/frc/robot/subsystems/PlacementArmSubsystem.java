// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import cwtech.util.RopeSensor;
import cwtech.util.SolenoidV2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class PlacementArmSubsystem extends SubsystemBase {

    public static final double kArmRotatorP = 0.1;
    public static final double kArmRotatorI = 0;
    public static final double kArmRotatorD = 0;
    public static final double kArmExtensionP = 0.1;
    public static final double kArmExtensionI = 0;
    public static final double kArmExtensionD = 0;

    CANSparkMax m_armRotatorMotor = new CANSparkMax(RobotMap.kArmRotatorSparkMaxMotor, MotorType.kBrushless);
    CANSparkMax m_armExtensionMotor = new CANSparkMax(RobotMap.kArmExtensionSparkMaxMotor, MotorType.kBrushless);
    DigitalInput m_armRotatorEncoderDigitalInput = new DigitalInput(RobotMap.kRotatorArmEncoderPulseWidthDIO);
    DutyCycle m_armRotatorEncoderDutyCycle = new DutyCycle(m_armRotatorEncoderDigitalInput);
    Spark m_gripVacuumMotor = new Spark(RobotMap.kGripVacuumSparkMotor);
    SolenoidV2 m_armVacuumRelease1 = new SolenoidV2(RobotMap.kArmVacuumRelease1);
    SolenoidV2 m_armVacuumRelease2 = new SolenoidV2(RobotMap.kArmVacuumRelease2);
    SolenoidV2 m_armVacuumRelease3 = new SolenoidV2(RobotMap.kArmVacuumRelease3);

    RopeSensor m_ropeSensor = new RopeSensor(RobotMap.kRopeEncoderDigIO);
    PIDController m_armExtensionMotorPidController = new PIDController(kArmExtensionP, kArmExtensionI, kArmExtensionD);
    double m_lastArmExtensionPosition = 0; // Arm starts fully retracted

    PIDController m_armRotatorMotorPidController = new PIDController(kArmRotatorP, kArmRotatorI, kArmRotatorD);

    static double kDegreesPerRevolution = 360.0 / 4096.0;

    private int m_ticks = 0;

    // ** Creates a new PlacementArmSubsystem. */
    public PlacementArmSubsystem() {
        m_armRotatorMotorPidController.setP(kArmRotatorP);
        m_armRotatorMotorPidController.setI(kArmRotatorI);
        m_armRotatorMotorPidController.setD(kArmRotatorD);

        m_armExtensionMotorPidController.setP(kArmExtensionP);
        m_armExtensionMotorPidController.setI(kArmExtensionI);
        m_armExtensionMotorPidController.setD(kArmExtensionD);

        smartDashboardInit();
    }

    public void setupRopeSensor(Robot robot) {
        m_ropeSensor.setup(robot);
    }

    public void smartDashboardInit() {
        SmartDashboard.putNumber(getName() + "/Arm Rotator P", m_armExtensionMotorPidController.getP());
        SmartDashboard.putNumber(getName() + "/Arm Rotator I", m_armExtensionMotorPidController.getI());
        SmartDashboard.putNumber(getName() + "/Arm Rotator D", m_armExtensionMotorPidController.getD());
        SmartDashboard.putNumber(getName() + "/Arm Extension P", m_armExtensionMotorPidController.getP());
        SmartDashboard.putNumber(getName() + "/Arm Extension I", m_armExtensionMotorPidController.getI());
        SmartDashboard.putNumber(getName() + "/Arm Extension D", m_armExtensionMotorPidController.getD());
    }

    public void smartDashboardUpdate() {
        SmartDashboard.putNumber(getName() + "/Arm Rotator Position", getArmAngle());
        m_armRotatorMotorPidController.setP(SmartDashboard.getNumber(getName() + "/Arm Rotator P", kArmRotatorP));
        m_armRotatorMotorPidController.setI(SmartDashboard.getNumber(getName() + "/Arm Rotator I", kArmRotatorI));
        m_armRotatorMotorPidController.setD(SmartDashboard.getNumber(getName() + "/Arm Rotator D", kArmRotatorD));
        m_armExtensionMotorPidController.setP(SmartDashboard.getNumber(getName() + "/Arm Extension P", kArmExtensionP));
        m_armExtensionMotorPidController.setI(SmartDashboard.getNumber(getName() + "/Arm Extension I", kArmExtensionI));
        m_armExtensionMotorPidController.setD(SmartDashboard.getNumber(getName() + "/Arm Extension", kArmExtensionD));
    }

    public void setArmDegrees(double degrees) {
        m_armRotatorMotorPidController.setSetpoint(degrees);
    }

    public void stopArmExtensionMotor() {
        m_armExtensionMotor.set(0.0);
        m_armExtensionMotorPidController.setSetpoint(getArmExtensionPosition());
    }

    public boolean isArmExtensionAtSetpoint() {
        return m_armExtensionMotorPidController.atSetpoint();
    }

    public boolean isArmRotatorAtSetpoint() {
        return m_armRotatorMotorPidController.atSetpoint();
    }

    public void stopArmRotatorMotor() {
        m_armRotatorMotor.set(0.0);
        m_armExtensionMotorPidController.setSetpoint(getArmAngle());
    }

    public double getArmAngle() {
        return m_armRotatorEncoderDutyCycle.getOutput() * 360;
    }

    public void setArmExtensionPosition(double distance) {
        m_armExtensionMotor.set(0); // stop motor so us modifying the going forward doesn't screw things up

        m_ropeSensor.setGoingForward(distance > m_lastArmExtensionPosition);

        m_armExtensionMotorPidController.setSetpoint(distance);
    }

    public double getArmExtensionPosition() {
        return m_ropeSensor.getTicks();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        SmartDashboard.putNumber(getName() + "/Arm Rotation Encoder Position", getArmAngle());

        m_ticks++;
        if (m_ticks % 15 != 3)
            return;

        // smartDashboardUpdate();

        // m_armExtensionMotor.set(m_armExtensionMotorPidController.calculate(getArmExtensionPosition()));
        // m_armRotatorMotor.set(m_armRotatorMotorPidController.calculate(getArmAngle()));
    }

    public void manipulateVacuumRelease(boolean release) {
        manipulateVacuumSolenoids(release);
        if (release) {
            manipulateVacuumMotors(false);
        }
    }

    public void engageVacuum() {
        manipulateVacuumSolenoids(false);
        manipulateVacuumMotors(true);
    }

    private void manipulateVacuumSolenoids(boolean newState)
    {
        m_armVacuumRelease1.set(newState);
        m_armVacuumRelease2.set(newState);
        m_armVacuumRelease3.set(newState);
    }

    private void manipulateVacuumMotors(boolean evacuate)
    {
        var speed = evacuate ? 1.0 : 0.0;
        m_gripVacuumMotor.set(speed);
    }
}
