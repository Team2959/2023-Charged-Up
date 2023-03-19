// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import cwtech.util.SolenoidV2;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IntakeSubsystem extends SubsystemBase
{
  private SolenoidV2 m_intakeArm = new SolenoidV2(RobotMap.kIntakeArm);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
  }

  public boolean intakeIsDown() {
    return m_intakeArm.get();
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

  public void turnOnIntake() {
    m_intakeArm.set(true);
  }

  public void turnOffIntake() {
    m_intakeArm.set(false);
  }
}
