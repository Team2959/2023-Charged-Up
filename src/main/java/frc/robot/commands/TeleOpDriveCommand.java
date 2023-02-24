// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TeleOpDriveCommand extends CommandBase {
    private DriveSubsystem m_driveSubsystem;
    private Supplier<Double> m_xJoystickSupplier;
    private Supplier<Double> m_yJoystickSupplier;
    private Supplier<Double> m_turnJoystickSupplier;
    private Supplier<Boolean> m_isTeleopEnabled;

    /** Creates a new TeleOpDriveCommand. */
    public TeleOpDriveCommand(DriveSubsystem driveSubsystem,
            Supplier<Double> xJoystick, Supplier<Double> yJoystick, Supplier<Double> turnJoystick, Supplier<Boolean> isTeleopEnabled) {
        m_driveSubsystem = driveSubsystem;

        m_xJoystickSupplier = xJoystick;
        m_yJoystickSupplier = yJoystick;
        m_turnJoystickSupplier = turnJoystick;
        m_isTeleopEnabled = isTeleopEnabled;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_isTeleopEnabled.get())
            m_driveSubsystem.drive(m_xJoystickSupplier.get(), m_yJoystickSupplier.get(), m_turnJoystickSupplier.get(),
                    true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.drive(0, 0, 0, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
