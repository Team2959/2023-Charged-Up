package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class ReverseAllIntakeCommand extends InstantCommand {
    IntakeSubsystem m_intakeSubsystem;
    boolean m_exceptFlipper;

    public ReverseAllIntakeCommand(IntakeSubsystem intakeSubsystem, boolean exceptFlipper) {
        m_intakeSubsystem = intakeSubsystem;
        m_exceptFlipper = exceptFlipper;
    }

    @Override
    public void initialize() {
        m_intakeSubsystem.reverseAll(m_exceptFlipper);
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.stopIntake();
    }
}
