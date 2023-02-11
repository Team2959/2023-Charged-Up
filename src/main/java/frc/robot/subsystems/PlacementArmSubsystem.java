// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class PlacementArmSubsystem extends SubsystemBase
{
    public enum ExtensionPosition
    {
        FullyRetracted,
        HalfExtended, 
        FullyExtented,
    };

    CANSparkMax m_armRotatorMotor = new CANSparkMax(RobotMap.kArmRotatorSparkMaxMotor, MotorType.kBrushless);
    CANSparkMax m_armExtensionmotor = new CANSparkMax(RobotMap.kArmExtensionSparkMaxMotor, MotorType.kBrushless);
    SparkMaxPIDController m_armRotatorMotorPidController = m_armRotatorMotor.getPIDController();
    SparkMaxAbsoluteEncoder m_armRotatorAbsoluteEncoder = (SparkMaxAbsoluteEncoder) m_armRotatorMotor
            .getAlternateEncoder(4096);
    Spark m_gripVacuumMotor1 = new Spark(RobotMap.kGripVacuum1SparkMotor);
    Spark m_gripVacuumMotor2 = new Spark(RobotMap.kGripVacuum2SparkMotor);
    Spark m_gripVacuumMotor3 = new Spark(RobotMap.kGripVacuum3SparkMotor);
    Solenoid m_armVacuumRelease = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.kArmVacuumRelease);

    // ** Creates a new PlacementArmSubsystem. */
    public PlacementArmSubsystem()
    {
        // TODO Pids for armRotatorMotorPidController and also smart motion stuff
        
        m_armRotatorMotorPidController.setFeedbackDevice(m_armRotatorAbsoluteEncoder);
    }

    public void setArmDegrees(double degrees)
    {
        m_armRotatorMotorPidController.setReference(degrees, ControlType.kSmartMotion);
    }
    
    public double getArmAngle() {

      
    }

    public void engageVacuum()
    {
        m_armVacuumRelease.set(false);
        m_gripVacuumMotor1.set(1.0);
        m_gripVacuumMotor2.set(1.0);
        m_gripVacuumMotor3.set(1.0);
    }

    public void disengageVacuum()
    {
        m_gripVacuumMotor1.set(0.0);
        m_gripVacuumMotor2.set(0.0);
        m_gripVacuumMotor3.set(0.0);
        m_armVacuumRelease.set(true);
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }
}
