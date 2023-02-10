// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PlacementArmSubsystem;

public class SetRotationPlacementArmCommand extends CommandBase
{
  /** Creates a new PlacementArmCommand. */

  private PlacementArmSubsystem m_PlacementArmSubsystem;
  private Rotation2d m_angle;

  public SetRotationPlacementArmCommand(PlacementArmSubsystem placementArmSubsystem, Rotation2d Angle)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    m_PlacementArmSubsystem = placementArmSubsystem;
    m_angle = Angle;

    addRequirements(placementArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_PlacementArmSubsystem.setArmDegrees(m_angle.getDegrees());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return true;
  }
}
