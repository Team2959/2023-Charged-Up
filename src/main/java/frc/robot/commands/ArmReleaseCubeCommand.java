package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmGamePieceControlSubsystem;

public class ArmReleaseCubeCommand extends SequentialCommandGroup {
    public ArmReleaseCubeCommand(ArmGamePieceControlSubsystem gamePieceControlSubsystem) {
        addCommands(new ArmVacuumReleaseCommand(gamePieceControlSubsystem));
    }
}
