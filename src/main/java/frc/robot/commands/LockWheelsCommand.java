package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class LockWheelsCommand extends CommandBase {
    DriveSubsystem m_DriveSubsystem;

    public LockWheelsCommand(DriveSubsystem driveSubsystem) {
        m_DriveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        m_DriveSubsystem.turn180();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
