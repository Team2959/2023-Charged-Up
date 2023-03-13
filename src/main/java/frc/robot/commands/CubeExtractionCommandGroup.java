// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmRotationSubsystem;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmGamePieceControlSubsystem;
import frc.robot.subsystems.ArmGamePieceControlSubsystem.GamePieceType;

public class CubeExtractionCommandGroup extends SequentialCommandGroup {
    public CubeExtractionCommandGroup(ArmRotationSubsystem armRotationSubsystem,
        ArmExtensionSubsystem armExtensionSubsystem,
        ArmGamePieceControlSubsystem armGamePieceControlSubsystem)
    {
        addCommands(
                new InstantCommand(() -> armGamePieceControlSubsystem.gamePiecePickup(GamePieceType.Cube)),
                new ArmExtentionCommand(armExtensionSubsystem, 25),
                new ArmRotationCommand(armRotationSubsystem, 90));
    }
}
