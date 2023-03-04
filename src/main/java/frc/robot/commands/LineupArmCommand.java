// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
      PlacementArmSubsystem.GamePieceType gamePieceType,
      ArmPositioningType positioningType)
  {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    double distance = 0;
    double angle = 0;
    switch (positioningType)
    {
      case High:
        distance = 40; //Inches
        if (gamePieceType == PlacementArmSubsystem.GamePieceType.Cone)
          angle = 90;
        else
          angle = 85;
        break;
      case Mid:
        distance = 23;
        if (gamePieceType == PlacementArmSubsystem.GamePieceType.Cone)
          angle = 60;
        else
          angle = 55;
        break;
      case Low:
        distance = 6;
        angle = 25;
        break;
    }
    
    addCommands(new ArmRotationCommand(placementArmSubsystem, angle));
    addCommands(new ArmExtentionCommand(placementArmSubsystem, distance));
  }
}
