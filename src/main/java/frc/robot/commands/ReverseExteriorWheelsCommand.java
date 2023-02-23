package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class ReverseExteriorWheelsCommand extends InstantCommand {
    IntakeSubsystem m_IntakeSubsystem;

    public ReverseExteriorWheelsCommand(IntakeSubsystem intakeSubsystem) {
        m_IntakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        m_IntakeSubsystem.reverseFront();
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakeSubsystem.stopIntake();
    }
}
