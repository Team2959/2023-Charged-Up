// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.RunPathCommand;
import frc.robot.commands.TeleOpDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import cwtech.util.Conditioning;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
    public final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

    private Joystick m_leftJoystick = new Joystick(RobotMap.kLeftJoystick);
    private Joystick m_rightJoystick = new Joystick(RobotMap.kRightJoystick);

    private Conditioning m_driveXConditioning = new Conditioning();
    private Conditioning m_driveYConditioning = new Conditioning();
    private Conditioning m_turnConditioning = new Conditioning();
    private double m_governer = 0.5;

    public double getDriveXInput() {
        // We getY() here because of the FRC coordinate system being turned 90 degrees
        return m_driveXConditioning.condition(m_leftJoystick.getY()) * DriveSubsystem.kMaxSpeedMetersPerSecond * m_governer;
    }

    public double getDriveYInput() {
        // We getX() here becasuse of the FRC coordinate system being turned 90 degrees
        return m_driveYConditioning.condition(m_leftJoystick.getX()) * DriveSubsystem.kMaxSpeedMetersPerSecond * m_governer;
    }

    public double getTurnInput() {
        return m_turnConditioning.condition(m_rightJoystick.getX()) * DriveSubsystem.kMaxAngularSpeedRadiansPerSecond;
    }

    public RobotContainer() {
        // Setup of conditioning calculations
        m_driveXConditioning.setDeadband(0.18);
        m_driveXConditioning.setExponent(1.7);
        m_driveYConditioning.setDeadband(0.15);
        m_driveYConditioning.setExponent(1.4);
        m_turnConditioning.setDeadband(0.2);
        m_turnConditioning.setExponent(1.4);

        configureBindings();
    }

    private void configureBindings() {
        m_driveSubsystem.setDefaultCommand(new TeleOpDriveCommand(m_driveSubsystem, () -> getDriveXInput(),
                () -> getDriveYInput(), () -> getTurnInput()));
    }

    public Command getAutonomousCommand() {
        return new RunPathCommand(m_driveSubsystem, "basic");
    }
}
