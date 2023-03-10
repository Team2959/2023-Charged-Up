package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.PlacementArmSubsystem;

public class BumpCubeCommand extends SequentialCommandGroup {
    public BumpCubeCommand(PlacementArmSubsystem placementArmSubsystem) {
        addCommands(new ArmExtentionCommand(placementArmSubsystem, 0));
        addCommands(new ArmRotationCommand(placementArmSubsystem, PlacementArmSubsystem.kArmHomePosition + 40));
        addCommands(new ArmRotationCommand(placementArmSubsystem, PlacementArmSubsystem.kArmHomePosition));
    }
}
