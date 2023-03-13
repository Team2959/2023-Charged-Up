// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmRotationSubsystem;

// ToDo? make parallel for improved speed?
public class ArmToLoadingCommand extends SequentialCommandGroup {
    public ArmToLoadingCommand(ArmRotationSubsystem armRotationSubsystem, ArmExtensionSubsystem armExtensionSubsystem) {
        addCommands(
                new ArmExtentionCommand(armExtensionSubsystem, 0),
                new ArmRotationCommand(armRotationSubsystem, ArmRotationSubsystem.kArmHomePosition + 10));
    }
}
