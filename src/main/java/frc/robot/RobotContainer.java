// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArmReleaseConeCommand;
import frc.robot.commands.ArmToLoadingCommand;
import frc.robot.commands.ArmVacuumReleaseCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.BumpArmRotationCommand;
import frc.robot.commands.LineupArmCommand;
import frc.robot.commands.LockWheelsCommand;
import frc.robot.commands.TeleOpDriveCommand;
import frc.robot.commands.TestArmExtensionCommand;
import frc.robot.commands.TestArmRotationCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.ArmPositioninInfo.ArmPositioningType;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmGamePieceControlSubsystem;
import frc.robot.subsystems.ArmRotationSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.ArmGamePieceControlSubsystem.GamePieceType;
import frc.robot.subsystems.ArmGamePieceControlSubsystem.UnloadType;
import cwtech.util.Conditioning;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
    private static double kDriveYExponent = 2; // 1.4;
    private static double kDriveXExponent = 2; // 1.7;

    public final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    public final ArmGamePieceControlSubsystem m_armGamePieceSubsystem = new ArmGamePieceControlSubsystem();
    public final ArmRotationSubsystem m_armRotationSubsystem = new ArmRotationSubsystem();
    public final ArmExtensionSubsystem m_armExtensionSubsystem = new ArmExtensionSubsystem();
    public final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
    public final Vision m_vision = new Vision();
    public final SendableChooser<Command> m_autoChooser = Autos.sendableChooser(this);

    Robot m_robot;

    Joystick m_leftJoystick = new Joystick(RobotMap.kLeftJoystick);
    Joystick m_rightJoystick = new Joystick(RobotMap.kRightJoystick);
    Joystick m_buttonBox = new Joystick(RobotMap.kButtonBox);

    // right driver joystick buttons
    JoystickButton m_intakeButton = new JoystickButton(m_rightJoystick, RobotMap.kRightToggleIntakeButton);
    JoystickButton m_armReleaseButtonRT = new JoystickButton(m_rightJoystick, RobotMap.kRightTriggerFire);
    JoystickButton m_lockWheeButton = new JoystickButton(m_rightJoystick, RobotMap.kRightLockWheels);
    JoystickButton m_balanceJoystickButton = new JoystickButton(m_rightJoystick, RobotMap.kRightBalance);

    // left driver joystick buttons

    // co-pilot box buttons
    JoystickButton m_armReleaseButton = new JoystickButton(m_buttonBox, RobotMap.kArmReleaseButton);

    JoystickButton m_highGamePieceButton = new JoystickButton(m_buttonBox, RobotMap.kHighGamePieceButton);
    JoystickButton m_midGamePieceButton = new JoystickButton(m_buttonBox, RobotMap.kMidGamePieceButton);
    JoystickButton m_lowGamePieceButton = new JoystickButton(m_buttonBox, RobotMap.kLowGamePieceButton);
    JoystickButton m_wallLineupHoriz = new JoystickButton(m_buttonBox, RobotMap.kLineUpWallGamePieceButton);
    JoystickButton m_groundPickupButtton = new JoystickButton(m_buttonBox, RobotMap.kGroundPickupButton);
    

    JoystickButton m_bumpArmAngleUp = new JoystickButton(m_buttonBox, RobotMap.kBumpArmAngleUpButton);
    JoystickButton m_bumpArmAngleDown = new JoystickButton(m_buttonBox, RobotMap.kBumpArmAngleDownButton);
    JoystickButton m_frontUnloadButton = new JoystickButton(m_buttonBox, RobotMap.kFrontUnloadButton);
    JoystickButton m_backUnloadButton = new JoystickButton(m_buttonBox, RobotMap.kBackUnloadButton);

    JoystickButton m_returnArmToLoadingButton = new JoystickButton(m_buttonBox, RobotMap.kReturnArmToHomeButton);

    JoystickButton m_gamePieceCubeButton = new JoystickButton(m_buttonBox, RobotMap.kGamePieceCubeButton);
    JoystickButton m_gamePieceConeButton = new JoystickButton(m_buttonBox, RobotMap.kGamePieceConeButton);

    JoystickButton m_testButtonExtension = new JoystickButton(m_rightJoystick, RobotMap.kTestButtonExtension);
    JoystickButton m_testButtonRotation = new JoystickButton(m_rightJoystick, RobotMap.kTestButtonRotation);

    Conditioning m_driveXConditioning = new Conditioning();
    Conditioning m_driveYConditioning = new Conditioning();
    Conditioning m_turnConditioning = new Conditioning();
    double m_speedMultiplier = 0.70;

    public SendableChooser<ArmGamePieceControlSubsystem.GamePieceType> m_preloadedPieceChooser = new SendableChooser<>();

    /**
     * The container form the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(Robot robot) {
        m_robot = robot;
        m_driveXConditioning.setDeadband(0.15);
        m_driveXConditioning.setExponent(kDriveXExponent);
        m_driveYConditioning.setDeadband(0.15);
        m_driveYConditioning.setExponent(kDriveYExponent);
        m_turnConditioning.setDeadband(0.2);
        m_turnConditioning.setExponent(1.4);

        m_preloadedPieceChooser.addOption("Cone", GamePieceType.Cone);
        m_preloadedPieceChooser.addOption("Cube", GamePieceType.Cube);
        SmartDashboard.putData("Auto/Preloaded Piece", m_preloadedPieceChooser);
        SmartDashboard.putData("CS", CommandScheduler.getInstance());
        SmartDashboard.putData("Auto/Routine", m_autoChooser);

        configureBindings();
        smartDashboardInit();
        registerSmartDashboardCalls();
    }

    public GamePieceType getPreloadedGamePieceType() {
        return m_preloadedPieceChooser.getSelected();
    }

    public void smartDashboardInit() {
        SmartDashboard.putNumber("Test Arm Extension Target Position", 0);
        SmartDashboard.putNumber("Test Arm Rotation Target Position", 60);
        // SmartDashboard.putNumber("Drive X/Exponent", kDriveXExponent);
        // SmartDashboard.putNumber("Drive Y/Exponent", kDriveYExponent);

        m_armRotationSubsystem.smartDashboardInit();
        m_armExtensionSubsystem.smartDashboardInit();
        m_driveSubsystem.smartDashboardInit();
    }

    public void registerSmartDashboardCalls() {
        m_robot.addPeriodic(() -> {
            m_driveSubsystem.smartDashboardUpdate();
            smartDashboardUpdate();
        }, 1, 0.502);
        // m_robot.addPeriodic(() -> {
        //     m_armRotationSubsystem.smartDashboardUpdate();
        //     m_armExtensionSubsystem.smartDashboardUpdate();
        // }, 1, 0.303);

    }

    public void teleOpInit() {
        SmartDashboard.putNumber("Speed Multiplier", m_speedMultiplier);
    }

    public void smartDashboardUpdate() {
        // restore if give driver/co-pilot more speed multiplier control
        // m_speedMultiplier = Math.max(0.5, Math.abs(m_leftJoystick.getThrottle()));
        // SmartDashboard.putNumber("Speed Multiplier", m_speedMultiplier);
        m_speedMultiplier = SmartDashboard.getNumber("Speed Multiplier", m_speedMultiplier);

        // kDriveXExponent = SmartDashboard.getNumber("Drive X/Exponent", kDriveXExponent);
        // kDriveYExponent = SmartDashboard.getNumber("Drive Y/Exponent", kDriveYExponent);
        // m_driveXConditioning.setExponent(kDriveXExponent);
        // m_driveYConditioning.setExponent(kDriveYExponent);
    }

    private void configureBindings() {
        m_driveSubsystem.setDefaultCommand(new TeleOpDriveCommand(m_driveSubsystem,
                () -> getDriveXInput(), () -> getDriveYInput(), () -> getTurnInput(), () -> m_robot.isTeleopEnabled()));

        m_intakeButton.onTrue(new InstantCommand(() -> m_intakeSubsystem.toggleIntakeSubsystem()));

        m_gamePieceConeButton.onTrue(new InstantCommand(() ->
            m_armGamePieceSubsystem.gamePiecePickup(GamePieceType.Cone)));
        m_gamePieceCubeButton.onTrue(new InstantCommand(() ->
            m_armGamePieceSubsystem.gamePiecePickup(GamePieceType.Cube)));

        m_wallLineupHoriz.onTrue(new LineupArmCommand(
                m_armRotationSubsystem, m_armExtensionSubsystem, m_armGamePieceSubsystem,
                ArmPositioningType.WallHorizLineup));

        m_lockWheeButton.whileTrue(new LockWheelsCommand(m_driveSubsystem));

        m_highGamePieceButton.onTrue(new LineupArmCommand(
                m_armRotationSubsystem, m_armExtensionSubsystem, m_armGamePieceSubsystem,
                ArmPositioningType.High));
        m_midGamePieceButton.onTrue(new LineupArmCommand(
                m_armRotationSubsystem, m_armExtensionSubsystem, m_armGamePieceSubsystem,
                ArmPositioningType.Mid));
        m_lowGamePieceButton.onTrue(new LineupArmCommand(
                m_armRotationSubsystem, m_armExtensionSubsystem, m_armGamePieceSubsystem,
                ArmPositioningType.Low));
                
        m_groundPickupButtton.onTrue(new LineupArmCommand(
                m_armRotationSubsystem, m_armExtensionSubsystem, m_armGamePieceSubsystem, 
                ArmPositioningType.FloorPickup));

        var armReleaseCommand = Commands.either(
                new ArmReleaseConeCommand(m_armGamePieceSubsystem, m_armRotationSubsystem),
                new ArmVacuumReleaseCommand(m_armGamePieceSubsystem),
                () -> m_armGamePieceSubsystem.getGamePieceType() == GamePieceType.Cone);
        m_armReleaseButton.whileTrue(armReleaseCommand);
        m_armReleaseButtonRT.whileTrue(armReleaseCommand);

        m_bumpArmAngleUp.onTrue(new BumpArmRotationCommand(m_armRotationSubsystem, 2));
        m_bumpArmAngleDown.onTrue(new BumpArmRotationCommand(m_armRotationSubsystem, -2));

        m_frontUnloadButton.onTrue(new InstantCommand(() -> m_armGamePieceSubsystem.setUnloadType(UnloadType.Front)));
        m_backUnloadButton.onTrue(new InstantCommand(() -> m_armGamePieceSubsystem.setUnloadType(UnloadType.Back)));

        m_returnArmToLoadingButton.onTrue(new ArmToLoadingCommand(m_armRotationSubsystem, m_armExtensionSubsystem));
        m_balanceJoystickButton.whileTrue(new AutoBalanceCommand(m_driveSubsystem));

        m_testButtonExtension.onTrue(new TestArmExtensionCommand(m_armExtensionSubsystem));
        m_testButtonRotation.onTrue(new TestArmRotationCommand(m_armRotationSubsystem));
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
