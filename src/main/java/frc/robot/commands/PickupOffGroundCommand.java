package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.PlacementArmSubsystem;

public class PickupOffGroundCommand extends SequentialCommandGroup {
    public PickupOffGroundCommand(PlacementArmSubsystem placementArmSubsystem) {
        addCommands(
            new ArmExtentionCommand(placementArmSubsystem, 0),
            new ArmRotationCommand(placementArmSubsystem, 100 /* LOW */),
            new ArmExtentionCommand(placementArmSubsystem, 50)
        );
    }
}
