// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PlacementArmSubsystem;

public class ArmExtentionCommand extends CommandBase {
  /** Creates a new ArmExtentionCommand. */
private double m_targetDistance;
private PlacementArmSubsystem m_PlacementArmSubsystem;

  public ArmExtentionCommand(PlacementArmSubsystem placementArmSubsystem, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(placementArmSubsystem);
    m_PlacementArmSubsystem = placementArmSubsystem;
    m_targetDistance = distance;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_PlacementArmSubsystem.setArmExtensionPosition(m_targetDistance);

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
    double m_currentdistance = m_PlacementArmSubsystem.getArmExtensionPosition();

    return Math.abs(m_targetDistance - m_currentdistance) < 1; // The 1 is a placeholder for later
  }
}
