package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmPositioninInfo.ArmPositioningType;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmGamePieceControlSubsystem;
import frc.robot.subsystems.ArmRotationSubsystem;


public class PickupOffGroundCommand extends SequentialCommandGroup {
    public PickupOffGroundCommand(ArmRotationSubsystem armRotationSubsystem,
    ArmExtensionSubsystem armExtensionSubsystem,
    ArmGamePieceControlSubsystem armGamePieceControlSubsystem,
    ArmPositioningType positioningType)
    {
            addCommands(
                new ArmExtentionCommand(armExtensionSubsystem, 0));
            addCommands(new ArmRotationByGamePieceAndPositionCommand(armRotationSubsystem,
                armGamePieceControlSubsystem, positioningType));
            addCommands(new ArmExtensionByGamePieceAndPositionCommand(armExtensionSubsystem,
                armGamePieceControlSubsystem, positioningType));
    }
}
