// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArmRotationCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.TeleOpDriveCommand;
import frc.robot.commands.ToggleIntakeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PlacementArmSubsystem;
import cwtech.util.Conditioning;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer
{
    // The robot's subsystems and commands are defined here...
    private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    private final PlacementArmSubsystem m_PlacementArmSubsystem = new PlacementArmSubsystem();
    public final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

    private Joystick m_leftJoystick = new Joystick(RobotMap.kLeftJoystick);
    private Joystick m_rightJoystick = new Joystick(RobotMap.kRightJoystick);
    private Joystick m_buttonBox = new Joystick(RobotMap.kButtonBox);
    JoystickButton m_IntakeButton = new JoystickButton(m_rightJoystick, RobotMap.kToggleIntakeButton);
    JoystickButton m_releaseButton = new JoystickButton(m_buttonBox, RobotMap.kGamePeiceReleaseButton);
    JoystickButton m_highGamePieceButton = new JoystickButton(m_buttonBox, 1);
    JoystickButton m_midGamePieceButton = new JoystickButton(m_buttonBox, 2);
    JoystickButton m_lowGamePieceButton = new JoystickButton(m_buttonBox, 3);

    private Conditioning m_driveXConditioning = new Conditioning();
    private Conditioning m_driveYConditioning = new Conditioning();
    private Conditioning m_turnConditioning = new Conditioning();
    private double m_governer = 0.5;

    /**
     * The container form the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        LiveWindow.enableAllTelemetry();
        // Setup of conditioning calculations
        m_driveXConditioning.setDeadband(0.18);
        m_driveXConditioning.setExponent(1.7);
        m_driveYConditioning.setDeadband(0.15);
        m_driveYConditioning.setExponent(1.4);
        m_turnConditioning.setDeadband(0.2);
        m_turnConditioning.setExponent(1.4);

        configureBindings();
    }

    private void configureBindings()
    {
        m_driveSubsystem.setDefaultCommand(new TeleOpDriveCommand(m_driveSubsystem,
            () -> getDriveXInput(), () -> getDriveYInput(), () -> getTurnInput()));
        m_IntakeButton.onTrue(new ToggleIntakeCommand(m_IntakeSubsystem));
        m_highGamePieceButton.onTrue(new ArmRotationCommand(m_PlacementArmSubsystem, 90));
        m_midGamePieceButton.onTrue(new ArmRotationCommand(m_PlacementArmSubsystem, 60));
        m_lowGamePieceButton.onTrue(new ArmRotationCommand(m_PlacementArmSubsystem, 30));
    }
    
    public Command getAutonomousCommand()
    {
        return Autos.runPath("Basic", m_driveSubsystem);
    }

    public double getDriveXInput()
    {
        // We getY() here because of the FRC coordinate system being turned 90 degrees
        return m_driveXConditioning.condition(m_leftJoystick.getY())
            * DriveSubsystem.kMaxSpeedMetersPerSecond
            * m_governer;
    }

    public double getDriveYInput()
    {
        // We getX() here becasuse of the FRC coordinate system being turned 90 degrees
        return m_driveYConditioning.condition(m_leftJoystick.getX())
            * DriveSubsystem.kMaxSpeedMetersPerSecond
            * m_governer;
    }

    public double getTurnInput()
    {
        return m_turnConditioning.condition(m_rightJoystick.getX())
            * DriveSubsystem.kMaxAngularSpeedRadiansPerSecond;
    }
}
