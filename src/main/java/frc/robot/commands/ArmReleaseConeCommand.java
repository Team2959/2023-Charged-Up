package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmGamePieceControlSubsystem;
import frc.robot.subsystems.ArmRotationSubsystem;

public class ArmReleaseConeCommand extends SequentialCommandGroup {
    public ArmReleaseConeCommand(ArmGamePieceControlSubsystem gamePieceControlSubsystem, ArmRotationSubsystem armRotationSubsystem) {
        addCommands(
            new BumpArmRotationCommand(armRotationSubsystem, -5),
            new ArmVacuumReleaseCommand(gamePieceControlSubsystem)
        );
    }
}
