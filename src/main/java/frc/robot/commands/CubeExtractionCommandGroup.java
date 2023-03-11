// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PlacementArmSubsystem;

public class CubeExtractionCommandGroup extends SequentialCommandGroup {
    public CubeExtractionCommandGroup(IntakeSubsystem intakeSubsystem, PlacementArmSubsystem placementArmSubsystem) {
        addCommands(
                new InstantCommand(() -> placementArmSubsystem.cubePickUp()),
                new ArmExtentionCommand(placementArmSubsystem, 25),
                new InstantCommand(() -> intakeSubsystem.flipGamePiece()),
                new ArmRotationCommand(placementArmSubsystem, 90));
    }
}
