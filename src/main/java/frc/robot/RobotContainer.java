// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArmToLoadingCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.CubeExtractionCommandGroup;
import frc.robot.commands.DropIntakeOrientaterCommand;
import frc.robot.commands.LineupArmCommand;
import frc.robot.commands.LockWheelsCommand;
import frc.robot.commands.PickupOffGroundCommand;
import frc.robot.commands.ReverseAllIntakeCommand;
import frc.robot.commands.ReverseExteriorWheelsCommand;
import frc.robot.commands.TeleOpDriveCommand;
import frc.robot.commands.TestArmExtensionCommand;
import frc.robot.commands.TestArmRotationCommand;
import frc.robot.commands.ArmVacuumReleaseCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.ToggleIntakeCommand;
import frc.robot.commands.ArmPositioninInfo.ArmPositioningType;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmGamePieceControlSubsystem;
import frc.robot.subsystems.ArmRotationSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmGamePieceControlSubsystem.GamePieceType;
import cwtech.util.Conditioning;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    private static double kDriveYExponent = 2; //1.4;
    private static double kDriveXExponent = 2; //1.7;
    // The robot's subsystems and commands are defined here...
    public final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    public final ArmGamePieceControlSubsystem m_ArmGamePieceSubsystem = new ArmGamePieceControlSubsystem();
    public final ArmRotationSubsystem m_ArmRotationSubsystem = new ArmRotationSubsystem();
    public final ArmExtensionSubsystem m_ArmExtensionSubsystem = new ArmExtensionSubsystem();
    public final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

    public final SendableChooser<Command> m_autoChooser = Autos.sendableChooser(this);

    Robot m_robot;

    Joystick m_leftJoystick = new Joystick(RobotMap.kLeftJoystick);
    Joystick m_rightJoystick = new Joystick(RobotMap.kRightJoystick);
    Joystick m_buttonBox = new Joystick(RobotMap.kButtonBox);
    JoystickButton m_lockWheeButton = new JoystickButton(m_rightJoystick, 1);
    JoystickButton m_wallLineupHoriz = new JoystickButton(m_leftJoystick, 11);
    JoystickButton m_IntakeButton = new JoystickButton(m_rightJoystick, RobotMap.kToggleIntakeButton);
    JoystickButton m_armReleaseButton = new JoystickButton(m_buttonBox, RobotMap.kArmReleaseButton);
    JoystickButton m_highGamePieceButton = new JoystickButton(m_buttonBox, RobotMap.kHighGamePeiceButton);
    JoystickButton m_midGamePieceButton = new JoystickButton(m_buttonBox, RobotMap.kMidGamePeiceButton);
    JoystickButton m_lowGamePieceButton = new JoystickButton(m_buttonBox, RobotMap.kLowGamePeiceButton);
    JoystickButton m_returnArmToLoadingButton = new JoystickButton(m_buttonBox, RobotMap.kReturnArmToLoadingButton);
    JoystickButton m_reverseIntakeButton = new JoystickButton(m_buttonBox, RobotMap.kReverseIntakeButton);
    JoystickButton m_reverseExteriorIntakeButton = new JoystickButton(m_buttonBox,
            RobotMap.kReverseExteriorIntakeButton);
    JoystickButton m_gamePieceConeButton = new JoystickButton(m_leftJoystick, RobotMap.kGamePieceConeButton);
    JoystickButton m_cubeEjectionButton = new JoystickButton(m_leftJoystick, RobotMap.kCubeEjectionButton);
    JoystickButton m_gamePieceCubeButton = new JoystickButton(m_buttonBox, RobotMap.kGamePieceCubeButton);
    JoystickButton m_testButton = new JoystickButton(m_buttonBox, RobotMap.kTestButton);
    JoystickButton m_testButton2 = new JoystickButton(m_buttonBox, RobotMap.kTestButton2);
    JoystickButton m_balJoystickButton = new JoystickButton(m_rightJoystick, 11);

    // JoystickButton m_PickUpWallGamePieceButton = new JoystickButton(m_leftJoystick, RobotMap.kPickUpWallGamePieceButton);
    // JoystickButton m_LineUpWallGamePieceButton = new JoystickButton(m_leftJoystick, RobotMap.kLineUpWallGamePieceButton);
    JoystickButton m_groundPickupButtton = new JoystickButton(m_leftJoystick, RobotMap.kGroundPickupButton);
    

    Conditioning m_driveXConditioning = new Conditioning();
    Conditioning m_driveYConditioning = new Conditioning();
    Conditioning m_turnConditioning = new Conditioning();
    double m_speedMultiplier = 0.70;

    SendableChooser<ArmGamePieceControlSubsystem.GamePieceType> m_preloadedPieceChooser = new SendableChooser<>();

    /**
     * The container form the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(Robot robot) {
        m_robot = robot;
        // Setup of conditioning calculations
        m_driveXConditioning.setDeadband(0.15);
        m_driveXConditioning.setExponent(kDriveXExponent);
        m_driveYConditioning.setDeadband(0.15);
        m_driveYConditioning.setExponent(kDriveYExponent);
        m_turnConditioning.setDeadband(0.2);
        m_turnConditioning.setExponent(1.4);

        // m_PlacementArmSubsystem.setupRopeSensor(robot);

        m_preloadedPieceChooser.addOption("Cone", GamePieceType.Cone);
        m_preloadedPieceChooser.addOption("Cube", GamePieceType.Cube);
        SmartDashboard.putData("Auto/Preloaded Piece", m_preloadedPieceChooser);
        SmartDashboard.putData("CS", CommandScheduler.getInstance());
        SmartDashboard.putData("Auto/Routine", m_autoChooser);

        configureBindings();
        smartDashboardInit();
        registerSmartDashboardCalls();
    }

    public void smartDashboardInit() {
        SmartDashboard.putNumber("Test Arm Extension Target Position", 0);
        SmartDashboard.putNumber("Test Arm Rotation Target Position", 60);
        SmartDashboard.putNumber("Drive X/Exponent", kDriveXExponent);
        SmartDashboard.putNumber("Drive Y/Exponent", kDriveYExponent);

        m_IntakeSubsystem.smartDashboardInit();
        m_ArmRotationSubsystem.smartDashboardInit();
        m_ArmExtensionSubsystem.smartDashboardInit();
        m_driveSubsystem.smartDashboardInit();
    }

    public void registerSmartDashboardCalls() {
        m_robot.addPeriodic(() -> {
            m_driveSubsystem.smartDashboardUpdate();
            smartDashboardUpdate();
        }, 1, 0.502);
        m_robot.addPeriodic(() -> {
            m_IntakeSubsystem.smartDashboardUpdate();
            m_ArmRotationSubsystem.smartDashboardUpdate();
            m_ArmExtensionSubsystem.smartDashboardUpdate();
        }, 1, 0.303);
        kDriveXExponent = SmartDashboard.getNumber("Drive X/Exponent", kDriveXExponent);
        kDriveYExponent = SmartDashboard.getNumber("Drive Y/Exponent", kDriveYExponent);
        m_driveXConditioning.setExponent(kDriveXExponent);
        m_driveYConditioning.setExponent(kDriveYExponent);
    }

    public void teleOpInit() {
        SmartDashboard.putNumber("Speed Multiplier", m_speedMultiplier);
    }

    public void smartDashboardUpdate() {
        // Math.max(0.5, Math.abs(m_leftJoystick.getThrottle()));
        m_speedMultiplier = SmartDashboard.getNumber("Speed Multiplier", m_speedMultiplier);
    }

    private void configureBindings() {
        m_driveSubsystem.setDefaultCommand(new TeleOpDriveCommand(m_driveSubsystem,
                () -> getDriveXInput(), () -> getDriveYInput(), () -> getTurnInput(), () -> m_robot.isTeleopEnabled()));

        m_IntakeButton.onTrue(new ToggleIntakeCommand(m_IntakeSubsystem));

        m_gamePieceConeButton.onTrue(new InstantCommand(() -> m_ArmGamePieceSubsystem.gamePiecePickup(GamePieceType.Cone)));
        m_gamePieceCubeButton.onTrue(new InstantCommand(() -> m_ArmGamePieceSubsystem.gamePiecePickup(GamePieceType.Cube)));

        m_wallLineupHoriz.onTrue(new LineupArmCommand(
            m_ArmRotationSubsystem, m_ArmExtensionSubsystem, m_ArmGamePieceSubsystem,
            ArmPositioningType.WallHorizLineup));

        m_lockWheeButton.whileTrue(new LockWheelsCommand(m_driveSubsystem));

        // m_LineUpWallGamePieceButton.onTrue(new LineupArmCommand(m_PlacementArmSubsystem, ArmPositioningType.WallLineup));
        // m_PickUpWallGamePieceButton.onTrue(new LineupArmCommand(m_PlacementArmSubsystem, ArmPositioningType.WallPickup));

        m_highGamePieceButton.onTrue(new LineupArmCommand(
            m_ArmRotationSubsystem, m_ArmExtensionSubsystem, m_ArmGamePieceSubsystem,
            ArmPositioningType.High));
        m_midGamePieceButton.onTrue(new LineupArmCommand(
            m_ArmRotationSubsystem, m_ArmExtensionSubsystem, m_ArmGamePieceSubsystem,
            ArmPositioningType.Mid));
        m_lowGamePieceButton.onTrue(new LineupArmCommand(
            m_ArmRotationSubsystem, m_ArmExtensionSubsystem, m_ArmGamePieceSubsystem,
            ArmPositioningType.Low));

// - -> left
// + -> forward
        m_armReleaseButton.whileTrue(new ArmVacuumReleaseCommand(m_ArmGamePieceSubsystem));

        m_groundPickupButtton.onTrue(new PickupOffGroundCommand(m_ArmRotationSubsystem, m_ArmExtensionSubsystem));

        m_returnArmToLoadingButton.onTrue(new ArmToLoadingCommand(m_ArmRotationSubsystem, m_ArmExtensionSubsystem));
        m_balJoystickButton.whileTrue(new AutoBalanceCommand(m_driveSubsystem));

        new Trigger(() -> m_IntakeSubsystem.intakeIsDown() && m_IntakeSubsystem.gamePieceDetected())
                .whileTrue(new DropIntakeOrientaterCommand(m_IntakeSubsystem));

        m_reverseIntakeButton.whileTrue(new ReverseAllIntakeCommand(m_IntakeSubsystem));
        m_reverseExteriorIntakeButton.whileTrue(new ReverseExteriorWheelsCommand(m_IntakeSubsystem));
        m_cubeEjectionButton.onTrue(new CubeExtractionCommandGroup(m_ArmRotationSubsystem,
            m_ArmExtensionSubsystem, m_ArmGamePieceSubsystem));

        m_testButton.onTrue(new TestArmExtensionCommand(m_ArmExtensionSubsystem));
        m_testButton2.onTrue(new TestArmRotationCommand(m_ArmRotationSubsystem));
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }

    public double getDriveXInput() {
        // We getY() here because of the FRC coordinate system being turned 90 degrees
        return m_driveXConditioning.condition(-m_leftJoystick.getY())
                * DriveSubsystem.kMaxSpeedMetersPerSecond
                * m_speedMultiplier;
    }

    public double getDriveYInput() {
        // We getX() here becasuse of the FRC coordinate system being turned 90 degrees
        return m_driveYConditioning.condition(-m_leftJoystick.getX())
                * DriveSubsystem.kMaxSpeedMetersPerSecond
                * m_speedMultiplier;
    }

    public double getTurnInput() {
        return m_turnConditioning.condition(-m_rightJoystick.getX())
                * DriveSubsystem.kMaxAngularSpeedRadiansPerSecond;
    }
}
