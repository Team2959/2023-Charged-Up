package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmRotationSubsystem;

public class BumpArmRotationCommand extends InstantCommand {
    ArmRotationSubsystem m_armRotationSubsystem;
    double m_change;
    
    public BumpArmRotationCommand(ArmRotationSubsystem armRotationSubsystem, double change) {
        m_armRotationSubsystem = armRotationSubsystem;
        m_change = change;
        addRequirements(m_armRotationSubsystem);
    }

    @Override
    public void initialize() {
        m_armRotationSubsystem.setArmDegrees(m_armRotationSubsystem.lastArmRotationTarget() + m_change);
    }

    @Override
    public boolean isFinished() {
        // return m_armRotationSubsystem.isArmRotatorAtSetpoint();
        return true;
    }
}
