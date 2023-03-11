// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PlacementArmSubsystem;

public class ArmToLoadingCommand extends SequentialCommandGroup {
    public ArmToLoadingCommand(PlacementArmSubsystem placementArmSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(
                new InstantCommand(() -> intakeSubsystem.unflipGamePiece()),
                new ArmExtentionCommand(placementArmSubsystem, 0),
                new ArmRotationCommand(placementArmSubsystem, PlacementArmSubsystem.kArmHomePosition + 10));
    }
}
