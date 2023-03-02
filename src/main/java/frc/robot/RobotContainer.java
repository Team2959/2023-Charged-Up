// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArmToLoadingCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.DropIntakeOrientaterCommand;
import frc.robot.commands.FlipGamePieceUpCommand;
import frc.robot.commands.IntakeVacuumReleaseCommand;
import frc.robot.commands.LineupArmCommand;
import frc.robot.commands.ReverseAllIntakeCommand;
import frc.robot.commands.ReverseExteriorWheelsCommand;
import frc.robot.commands.TeleOpDriveCommand;
import frc.robot.commands.ArmVacuumReleaseCommand;
import frc.robot.commands.ToggleIntakeCommand;
import frc.robot.commands.LineupArmCommand.ArmPositioningType;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PlacementArmSubsystem;

import cwtech.util.Conditioning;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    private final PlacementArmSubsystem m_PlacementArmSubsystem = new PlacementArmSubsystem();
    public final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

    Robot m_robot;

    Joystick m_leftJoystick = new Joystick(RobotMap.kLeftJoystick);
    Joystick m_rightJoystick = new Joystick(RobotMap.kRightJoystick);
    Joystick m_buttonBox = new Joystick(RobotMap.kButtonBox);
    JoystickButton m_IntakeButton = new JoystickButton(m_rightJoystick, RobotMap.kToggleIntakeButton);
    JoystickButton m_armReleaseButton = new JoystickButton(m_buttonBox, RobotMap.kArmReleaseButton);
    JoystickButton m_intakeReleaseButton = new JoystickButton(m_buttonBox, RobotMap.kIntakeReleaseButton);
    JoystickButton m_highGamePieceButton = new JoystickButton(m_buttonBox, RobotMap.kHighGamePeiceButton);
    JoystickButton m_midGamePieceButton = new JoystickButton(m_buttonBox, RobotMap.kMidGamePeiceButton);
    JoystickButton m_lowGamePieceButton = new JoystickButton(m_buttonBox, RobotMap.kLowGamePeiceButton);
    JoystickButton m_returnArmToLoadingButton = new JoystickButton(m_buttonBox, RobotMap.kReturnArmToLoadingButton);
    JoystickButton m_reverseIntakeButton = new JoystickButton(m_buttonBox, RobotMap.kReverseIntakeButton);
    JoystickButton m_reverseExteriorIntakeButton = new JoystickButton(m_buttonBox, RobotMap.kReverseExteriorIntakeButton);
    JoystickButton m_testButton = new JoystickButton(m_buttonBox, RobotMap.kTestButton);

    Conditioning m_driveXConditioning = new Conditioning();
    Conditioning m_driveYConditioning = new Conditioning();
    Conditioning m_turnConditioning = new Conditioning();
    double m_speedMultiplier = 0.8;

    /**
     * The container form the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(Robot robot) {
        m_robot = robot;
        // Setup of conditioning calculations
        m_driveXConditioning.setDeadband(0.18);
        m_driveXConditioning.setExponent(1.7);
        m_driveYConditioning.setDeadband(0.15);
        m_driveYConditioning.setExponent(1.4);
        m_turnConditioning.setDeadband(0.2);
        m_turnConditioning.setExponent(1.4);

        SmartDashboard.putNumber("Speed Multiplier", m_speedMultiplier);

        configureBindings();
    }

    public void teleOpInit() {
        m_speedMultiplier = SmartDashboard.getNumber("Speed Multiplier", m_speedMultiplier);
    }

    private void configureBindings() {
        m_driveSubsystem.setDefaultCommand(new TeleOpDriveCommand(m_driveSubsystem,
                () -> getDriveXInput(), () -> getDriveYInput(), () -> getTurnInput(), () -> m_robot.isTeleopEnabled()));

        m_IntakeButton.onTrue(new ToggleIntakeCommand(m_IntakeSubsystem));

        m_highGamePieceButton.onTrue(new LineupArmCommand(m_PlacementArmSubsystem,
                m_IntakeSubsystem.getGamePieceType(), ArmPositioningType.High));
        m_midGamePieceButton.onTrue(new LineupArmCommand(m_PlacementArmSubsystem,
                m_IntakeSubsystem.getGamePieceType(), ArmPositioningType.Mid));
        m_lowGamePieceButton.onTrue(new LineupArmCommand(m_PlacementArmSubsystem,
                m_IntakeSubsystem.getGamePieceType(), ArmPositioningType.Low));
        
        m_testButton.onTrue(new InstantCommand(() -> {
            m_IntakeSubsystem.flipGamePiece();
            if(m_IntakeSubsystem.isVacuumEngaged()) {
                m_IntakeSubsystem.stopVacuum();
            }
            else {
                m_IntakeSubsystem.startVacuum();
            }
        }));

        m_armReleaseButton.whileTrue(new ArmVacuumReleaseCommand(m_PlacementArmSubsystem));
        m_intakeReleaseButton.whileTrue(new IntakeVacuumReleaseCommand(m_IntakeSubsystem));

        m_returnArmToLoadingButton.onTrue(new ArmToLoadingCommand(m_PlacementArmSubsystem));

        new Trigger(() -> m_IntakeSubsystem.intakeIsDown() && m_IntakeSubsystem.gamePieceDetected())
                .whileTrue(new DropIntakeOrientaterCommand(m_IntakeSubsystem));

        new Trigger(m_IntakeSubsystem::gamePieceIsReady).onTrue(new FlipGamePieceUpCommand(m_IntakeSubsystem));

        m_reverseIntakeButton.whileTrue(new ReverseAllIntakeCommand(m_IntakeSubsystem, false));
        m_reverseExteriorIntakeButton.whileTrue(new ReverseExteriorWheelsCommand(m_IntakeSubsystem));
    }

    public Command getAutonomousCommand() {
        return Autos.runPath("Basic", m_driveSubsystem);
    }

    public double getDriveXInput() {
        // return 0;
        // We getY() here because of the FRC coordinate system being turned 90 degrees
        return m_driveXConditioning.condition(-m_leftJoystick.getY())
                * DriveSubsystem.kMaxSpeedMetersPerSecond
                * m_speedMultiplier;
    }

    public double getDriveYInput() {
        // return 0;
        // We getX() here becasuse of the FRC coordinate system being turned 90 degrees
        return m_driveYConditioning.condition(m_leftJoystick.getX())
                * DriveSubsystem.kMaxSpeedMetersPerSecond
                * m_speedMultiplier;
    }

    public double getTurnInput() {
        // return 0;
        return m_turnConditioning.condition(m_rightJoystick.getX())
                * DriveSubsystem.kMaxAngularSpeedRadiansPerSecond;
    }
}
