package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalanceCommand extends CommandBase {
    DriveSubsystem m_DriveSubsystem;

    public AutoBalanceCommand(DriveSubsystem driveSubsystem) {
        m_DriveSubsystem = driveSubsystem;
        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void initialize() {
        m_DriveSubsystem.turnOnBalancing();
    }

    @Override
    public void end(boolean interrupted) {
        m_DriveSubsystem.turnOffBalancing();
        m_DriveSubsystem.drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
