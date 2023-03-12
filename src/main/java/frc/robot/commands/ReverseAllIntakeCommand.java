package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class ReverseAllIntakeCommand extends InstantCommand {
    IntakeSubsystem m_intakeSubsystem;

    public ReverseAllIntakeCommand(IntakeSubsystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        m_intakeSubsystem.reverseAll();
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.stopIntake();
    }
}
