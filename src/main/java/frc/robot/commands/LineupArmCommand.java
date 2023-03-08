// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmPositioninInfo.ArmPositioningType;
import frc.robot.subsystems.PlacementArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LineupArmCommand extends SequentialCommandGroup
{
  /** Creates a new LineupArmCommand. */
  public LineupArmCommand(PlacementArmSubsystem placementArmSubsystem,
      ArmPositioningType positioningType)
  {
    addCommands(new ArmRotationByArmPositionTypeCommand(placementArmSubsystem, positioningType));
    addCommands(new ArmExtentionCommand(placementArmSubsystem, ArmPositioninInfo.getArmDistance(positioningType)));
  }
}
