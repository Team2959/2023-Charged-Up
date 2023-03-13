package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmRotationSubsystem;

public class BumpCubeCommand extends SequentialCommandGroup {
    public BumpCubeCommand(ArmRotationSubsystem armRotationSubsystem,
        ArmExtensionSubsystem armExtensionSubsystem) {
        addCommands(new ArmExtentionCommand(armExtensionSubsystem, 0));
        addCommands(new ArmRotationCommand(armRotationSubsystem, ArmRotationSubsystem.kArmHomePosition + 40));
        addCommands(new ArmRotationCommand(armRotationSubsystem, ArmRotationSubsystem.kArmHomePosition));
    }
}
