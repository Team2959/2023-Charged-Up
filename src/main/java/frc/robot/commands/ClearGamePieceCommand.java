package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmRotationSubsystem;

public class ClearGamePieceCommand extends SequentialCommandGroup {
    public ClearGamePieceCommand(ArmRotationSubsystem armRotationSubsystem,
        ArmExtensionSubsystem armExtensionSubsystem) {
        addCommands(
            new ArmExtentionCommand(armExtensionSubsystem, 30),
            new ArmRotationCommand(armRotationSubsystem, ArmRotationSubsystem.kArmHomePosition + 20),
            new ArmRotationCommand(armRotationSubsystem, ArmRotationSubsystem.kArmHomePosition - 30));
    }
}
