package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class FlipGamePieceUpCommand extends SequentialCommandGroup {
    IntakeSubsystem m_IntakeSubsystem;

    public FlipGamePieceUpCommand(IntakeSubsystem intakeSubsystem) {
        addCommands(
            new InstantCommand(() -> m_IntakeSubsystem.startVacuum()),
            new WaitCommand(1.0),
            new InstantCommand(() -> m_IntakeSubsystem.flipGamePiece())
        );
    }
}
