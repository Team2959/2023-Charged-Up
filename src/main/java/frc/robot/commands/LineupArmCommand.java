// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.PlacementArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LineupArmCommand extends SequentialCommandGroup
{
  public enum ArmPositioningType
  {
    High,
    Mid,
    Low,
  };

  /** Creates a new LineupArmCommand. */
  public LineupArmCommand(PlacementArmSubsystem placementArmSubsystem,
      ArmPositioningType positioningType)
  {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    double distance = 0;
    switch (positioningType)
    {
      case High:
        distance = 115; 
        break;
      case Mid:
        distance = 50;
        break;
      case Low:
        distance = 30;
        break;
    }
    
    addCommands(new ArmRotationCommand(placementArmSubsystem, () -> placementArmSubsystem.getTargetGamePieceAngle(positioningType)));
    addCommands(new ArmExtentionCommand(placementArmSubsystem, distance));
  }
}
