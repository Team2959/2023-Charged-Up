// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmPositioninInfo.ArmPositioningType;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmGamePieceControlSubsystem;
import frc.robot.subsystems.ArmRotationSubsystem;

// ToDo?: Make parallel with extension constraints? -> improve speed
public class LineupArmCommand extends SequentialCommandGroup {
    public LineupArmCommand(ArmRotationSubsystem armRotationSubsystem,
            ArmExtensionSubsystem armExtensionSubsystem,
            ArmGamePieceControlSubsystem armGamePieceControlSubsystem,
            ArmPositioningType positioningType) {
        addCommands(new ArmRotationByGamePieceAndPositionCommand(armRotationSubsystem,
            armGamePieceControlSubsystem, positioningType));
        addCommands(new ArmExtensionByGamePieceAndPositionCommand(armExtensionSubsystem,
            armGamePieceControlSubsystem, positioningType));
    }
}
