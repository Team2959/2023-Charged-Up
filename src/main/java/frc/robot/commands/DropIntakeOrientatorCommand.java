package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class DropIntakeOrientatorCommand extends CommandBase {
    IntakeSubsystem m_intakeSubsystem;

    public DropIntakeOrientatorCommand(IntakeSubsystem intakeSubsystem) {
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        m_intakeSubsystem.dropOrientatorBar();
    }

    @Override
    public void end(boolean end) {
        m_intakeSubsystem.pullUpOrientatorBar();
    }

}
