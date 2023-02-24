// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PlacementArmSubsystem;

public class ArmVacuumReleaseCommand extends CommandBase {
  /** Creates a new ToggleArmVacuumCommand. */

  private PlacementArmSubsystem m_placementArmSubsystem;

  public ArmVacuumReleaseCommand(PlacementArmSubsystem placementArmSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_placementArmSubsystem = placementArmSubsystem;
    addRequirements(placementArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_placementArmSubsystem.manipulateVacuumRelease(true);  // release vacuum and shut off vacuum motors
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_placementArmSubsystem.manipulateVacuumRelease(false);  // re-engage vacuum solenoid
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // ToDo: maybe switch to TimedCommand and add a delay before re-engage
    return true;
  }
}
