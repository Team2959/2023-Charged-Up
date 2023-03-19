package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmRotationSubsystem;

public class PickupOffGroundCommand extends SequentialCommandGroup {
    public PickupOffGroundCommand(ArmRotationSubsystem armRotationSubsystem, ArmExtensionSubsystem armExtensionSubsystem) {
        addCommands(
                new ArmExtentionCommand(armExtensionSubsystem, 00),
                new ArmRotationCommand(armRotationSubsystem, 95 /* LOW */),
                new ArmExtentionCommand(armExtensionSubsystem, 40));
    }
}
