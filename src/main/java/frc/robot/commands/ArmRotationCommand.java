// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PlacementArmSubsystem;

public class ArmRotationCommand extends CommandBase
{
  /** Creates a new PlacementArmCommand. */

  private PlacementArmSubsystem m_PlacementArmSubsystem;
  private double m_angle;

  public ArmRotationCommand(PlacementArmSubsystem placementArmSubsystem, double angleInDegrees)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    m_PlacementArmSubsystem = placementArmSubsystem;
    m_angle = angleInDegrees;

    addRequirements(placementArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_PlacementArmSubsystem.setArmDegrees(m_angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // ToDo: need to stop motor
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    double currentAngle = m_PlacementArmSubsystem.getArmAngle();
    
    return Math.abs(m_angle - currentAngle) < 0.5;
  }
}
