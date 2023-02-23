package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class FlipperToggleCommand extends InstantCommand {
    IntakeSubsystem m_IntakeSubsystem;

    public FlipperToggleCommand(IntakeSubsystem intakeSubsystem) {
        m_IntakeSubsystem = intakeSubsystem;
    }    

    @Override
    public void initialize() {
        m_IntakeSubsystem.toggleFlipper();
    }
}
