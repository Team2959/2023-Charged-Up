// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import cwtech.util.RopeSensor;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class PlacementArmSubsystem extends SubsystemBase
{

    public static final double kArmRotatorP = 0;
    public static final double kArmRotatorI = 0;
    public static final double kArmRotatorD = 0;
    public static final double kArmRotatorIZone = 0;
    public static final double kArmRotatorFF = 0;
    public static final double kArmRotatorSmartMotionMaxVelocity = 0;
    public static final double kArmRotatorSmartMotionMaxAcceleration = 0;
    public static final double kArmExtensionP = 0;
    public static final double kArmExtensionI = 0;
    public static final double kArmExtensionD = 0;
    public static final double kArmExtensionIZone = 0;
    public static final double kArmExtensionFF = 0;
    public static final double kArmExtensionSmartMotionMaxVelocity = 0;
    public static final double kArmExtensionSmartMotionMaxAcceleration = 0;

    CANSparkMax m_armRotatorMotor = new CANSparkMax(RobotMap.kArmRotatorSparkMaxMotor, MotorType.kBrushless);
    CANSparkMax m_armExtensionMotor = new CANSparkMax(RobotMap.kArmExtensionSparkMaxMotor, MotorType.kBrushless);
    DigitalInput m_armRotatorEncoderDigitalInput = new DigitalInput(8);
    DutyCycle m_armRotatorEncoderDutyCycle = new DutyCycle(m_armRotatorEncoderDigitalInput);
    Spark m_gripVacuumMotor1 = new Spark(RobotMap.kGripVacuum1SparkMotor);
    Spark m_gripVacuumMotor2 = new Spark(RobotMap.kGripVacuum2SparkMotor);
    Spark m_gripVacuumMotor3 = new Spark(RobotMap.kGripVacuum3SparkMotor);
    Solenoid m_armVacuumRelease = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.kArmVacuumRelease);

    RopeSensor m_ropeSensor = new RopeSensor(7);
    PIDController m_armExtensionMotorPidController = new PIDController(kArmExtensionP, kArmExtensionI, kArmExtensionD);
    double m_lastArmExtensionPosition = 0; // Arm starts fully retracted

    PIDController m_armRotatorMotorPidController = new PIDController(kArmRotatorP, kArmRotatorI, kArmRotatorD);

    static double kDegreesPerRevolution = 360.0 / 4096.0;

    // ** Creates a new PlacementArmSubsystem. */
    public PlacementArmSubsystem()
    {
        m_armRotatorMotorPidController.setP(kArmRotatorP);
        m_armRotatorMotorPidController.setI(kArmRotatorI);
        m_armRotatorMotorPidController.setD(kArmRotatorD);
        // m_armRotatorMotorPidController.setIZone(kArmRotatorIZone);
        // m_armRotatorMotorPidController.setFF(kArmRotatorFF);
        // m_armRotatorMotorPidController.setSmartMotionMaxVelocity(kArmRotatorSmartMotionMaxVelocity, 0);
        // m_armRotatorMotorPidController.setSmartMotionMaxAccel(kArmRotatorSmartMotionMaxAcceleration, 0);
        
        
        m_armExtensionMotorPidController.setP(kArmExtensionP);
        m_armExtensionMotorPidController.setI(kArmExtensionI);
        m_armExtensionMotorPidController.setD(kArmExtensionD);
        // m_armExtensionMotorPidController.setIZone(kArmExtensionIZone);
        // m_armExtensionMotorPidController.setFF(kArmExtensionFF);
        // m_armExtensionMotorPidController.setSmartMotionMaxVelocity(kArmExtensionSmartMotionMaxVelocity, 0);
        // m_armExtensionMotorPidController.setSmartMotionMaxAccel(kArmExtensionSmartMotionMaxAcceleration, 0);
    }

    public void setupRopeSensor(Robot robot) {
        m_ropeSensor.setup(robot);
    }

    public void armSmartDashboardInit() {
        SmartDashboard.putNumber(getName() + "/Arm Rotator P", m_armExtensionMotorPidController.getP());
        SmartDashboard.putNumber(getName() + "/Arm Rotator I", m_armExtensionMotorPidController.getI());
        SmartDashboard.putNumber(getName() + "/Arm Rotator D", m_armExtensionMotorPidController.getD());
        // SmartDashboard.putNumber(getName() + "/Arm Rotator IZone", m_armExtensionMotorPidController.getIZone());
        // SmartDashboard.putNumber(getName() + "/Arm Extension FF", m_armExtensionMotorPidController.getFF());
        // SmartDashboard.putNumber(getName() + "/Arm Rotator Smart Motion Max Velocity", m_armExtensionMotorPidController.getSmartMotionMaxVelocity(0));
        // SmartDashboard.putNumber(getName() + "/Arm Rotator Smart Motion Acceleration", m_armExtensionMotorPidController.getSmartMotionMaxAccel(0));
        SmartDashboard.putNumber(getName() + "/Arm Extension P", m_armExtensionMotorPidController.getP());
        SmartDashboard.putNumber(getName() + "/Arm Extension I", m_armExtensionMotorPidController.getI());
        SmartDashboard.putNumber(getName() + "/Arm Extension D", m_armExtensionMotorPidController.getD());
        // SmartDashboard.putNumber(getName() + "/Arm Extension IZone", m_armExtensionMotorPidController.getIZone());
        // SmartDashboard.putNumber(getName() + "/Arm Extension FF", m_armExtensionMotorPidController.getFF());
        // SmartDashboard.putNumber(getName() + "/Arm Extension Smart Motion Max Velocity", m_armExtensionMotorPidController.getSmartMotionMaxVelocity(0));
        // SmartDashboard.putNumber(getName() + "/Arm Extension Smart Motion Acceleration", m_armExtensionMotorPidController.getSmartMotionMaxAccel(0));
        
    }

    public void armSmartDashboardUpdate() {
        SmartDashboard.putNumber(getName() + "/Arm Rotator Position", getArmAngle());
        m_armRotatorMotorPidController.setP (SmartDashboard.getNumber(getName() + "/Arm Rotator P", kArmRotatorP));
        m_armRotatorMotorPidController.setI (SmartDashboard.getNumber(getName() + "/Arm Rotator I", kArmRotatorI));
        m_armRotatorMotorPidController.setD (SmartDashboard.getNumber(getName() + "/Arm Rotator D", kArmRotatorD));
        // m_armRotatorMotorPidController.setIZone (SmartDashboard.getNumber(getName() + "/Arm Rotator IZone", kArmRotatorIZone));
        // m_armRotatorMotorPidController.setFF (SmartDashboard.getNumber(getName() + "/Arm Rotator FF", kArmRotatorFF));
        // m_armRotatorMotorPidController.setSmartMotionMaxVelocity(SmartDashboard.getNumber(getName() + "/Arm Rotator Smart Motion Velocity", kArmRotatorSmartMotionMaxVelocity), 0);
        // m_armRotatorMotorPidController.setSmartMotionMaxAccel(SmartDashboard.getNumber(getName() + "/Arm Rotator Smart Motion Acceleration", kArmRotatorSmartMotionMaxAcceleration), 0);
        m_armExtensionMotorPidController.setP (SmartDashboard.getNumber(getName() + "/Arm Extension P", kArmExtensionP));
        m_armExtensionMotorPidController.setI (SmartDashboard.getNumber(getName() + "/Arm Extension I", kArmExtensionI));
        m_armExtensionMotorPidController.setD (SmartDashboard.getNumber(getName() + "/Arm Extension", kArmExtensionD));
        // m_armExtensionMotorPidController.setIZone (SmartDashboard.getNumber(getName() + "/Arm Extension IZone", kArmExtensionIZone));
        // m_armExtensionMotorPidController.setFF (SmartDashboard.getNumber(getName() + "/Arm Extension FF", kArmExtensionFF));
        // m_armExtensionMotorPidController.setSmartMotionMaxVelocity(SmartDashboard.getNumber(getName() + "/Arm Extension Smart Motion Velocity", kArmRotatorSmartMotionMaxVelocity), 0);
        // m_armExtensionMotorPidController.setSmartMotionMaxAccel(SmartDashboard.getNumber(getName() + "/Arm Extension Smart Motion Acceleration", kArmRotatorSmartMotionMaxAcceleration), 0);
    }

    public void setArmDegrees(double degrees)
    { 
        // m_armRotatorMotorPidController.setReference(degrees, ControlType.kSmartMotion);
    }
    
    public double getArmAngle() {
      return m_armRotatorEncoderDutyCycle.getOutput() * 360;
    }

    public void setArmExtensionPosition(double distance) {
        double delta = distance - m_lastArmExtensionPosition;

        m_armExtensionMotor.set(0); // stop motor so us modifying the going forward
        if(distance > m_lastArmExtensionPosition) {
            m_ropeSensor.setGoingForward(true);
        } else {
            m_ropeSensor.setGoingForward(false);
        }

        m_armExtensionMotorPidController.setSetpoint(delta);
    }

    public double getArmExtensionPosition() {
        return m_ropeSensor.getTicks();
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run

        SmartDashboard.putNumber(getName() + "/Arm Rotation Encoder Position", getArmAngle());
    
        m_armExtensionMotor.set(m_armExtensionMotorPidController.calculate(getArmExtensionPosition()));
    }

    public void manipulateVacuumRelease(boolean release)
    {
        if (release)
        {
            m_armVacuumRelease.set(true);
            m_gripVacuumMotor1.set(0);
            m_gripVacuumMotor2.set(0);
            m_gripVacuumMotor3.set(0);
        }
        else
        {
            m_armVacuumRelease.set(false);
        }
    }

    public void engageVacuum()
    {
        m_armVacuumRelease.set(false);
        m_gripVacuumMotor1.set(1.0);
        m_gripVacuumMotor2.set(1.0);
        m_gripVacuumMotor3.set(1.0);
    }


    
}
