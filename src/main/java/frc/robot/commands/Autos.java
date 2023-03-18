// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmPositioninInfo.ArmPositioningType;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmGamePieceControlSubsystem;
import frc.robot.subsystems.ArmRotationSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmGamePieceControlSubsystem.GamePieceType;

public final class Autos {
    public static SendableChooser<Command> sendableChooser(RobotContainer container) {
        SendableChooser<Command> sendableChooser = new SendableChooser<>();
        sendableChooser.addOption("Nothing", new WaitCommand(0));
        sendableChooser.setDefaultOption("Place And Leave Left", placeAndLeaveLeft(container));
        sendableChooser.addOption("Place And Leave Right", placeAndLeaveRight(container));
        sendableChooser.addOption("Place And Leave And Balance", placeAndLeaveAndBalance(container));
        sendableChooser.addOption("Place And Balance", placeAndBalance(container));
        sendableChooser.addOption("Drive Only", runPath("Place Game Piece", container.m_driveSubsystem));
        return sendableChooser;
    }

    public static Command placeGamePiece(GamePieceType gamePieceType, DriveSubsystem driveSubsystem,
            ArmRotationSubsystem armRotationSubsystem,
            ArmExtensionSubsystem armExtensionSubsystem,
            ArmGamePieceControlSubsystem armGamePieceControlSubsystem,
            IntakeSubsystem intakeSubsystem) {
        Command readyPiece = Commands.sequence(
                new InstantCommand(() -> {
                    armGamePieceControlSubsystem.gamePiecePickup(gamePieceType);
                }),
                new WaitCommand(1), // remove?
                new ArmRotationCommand(armRotationSubsystem, 55), // remove?
                new WaitCommand(1), // reduce 0.25?
                new ArmExtentionCommand(armExtensionSubsystem, 0), // 25, so not as far?
                new LineupArmCommand(armRotationSubsystem, armExtensionSubsystem, armGamePieceControlSubsystem,
                        ArmPositioningType.High),
                new WaitCommand(0.5), // why wait? or reduce
                new ArmVacuumReleaseCommand(armGamePieceControlSubsystem),
                new WaitCommand(2)); // see if can reduce!! 0.5?
        return readyPiece.andThen(runPath("Place Game Piece", driveSubsystem))
                .andThen(new ArmToLoadingCommand(armRotationSubsystem, armExtensionSubsystem));
    }

    public static Command placeGamePieceAndBalance(GamePieceType gamePieceType, DriveSubsystem driveSubsystem,
            ArmRotationSubsystem armRotationSubsystem,
            ArmExtensionSubsystem armExtensionSubsystem,
            ArmGamePieceControlSubsystem armGamePieceControlSubsystem) {
        Command readyPiece = Commands.sequence(
                new InstantCommand(() -> {
                    armGamePieceControlSubsystem.gamePiecePickup(gamePieceType);
                }),
                new WaitCommand(1),
                new ArmRotationCommand(armRotationSubsystem, 55),
                new WaitCommand(1),
                new ArmExtentionCommand(armExtensionSubsystem, 0),
                new LineupArmCommand(armRotationSubsystem, armExtensionSubsystem, armGamePieceControlSubsystem,
                        ArmPositioningType.High),
                new WaitCommand(0.5),
                new ArmVacuumReleaseCommand(armGamePieceControlSubsystem),
                new WaitCommand(0.5));
        return readyPiece.andThen(runPath("Place Game Piece And Balance", driveSubsystem))
                .andThen(new ArmToLoadingCommand(armRotationSubsystem, armExtensionSubsystem))
                .andThen(new AutoBalanceCommand(driveSubsystem));
    }

    public static Command runPath(String name, HashMap<String, Command> events, DriveSubsystem driveSubsystem) {
        var trajectory = PathPlanner.loadPath(name, new PathConstraints(3, 2));
        SwerveAutoBuilder builder = new SwerveAutoBuilder(
                driveSubsystem::getPose,
                driveSubsystem::resetOdometry,
                driveSubsystem.getKinematics(),
                new PIDConstants(0.7, 0.0001, 0.0),
                new PIDConstants(0.1, 0.0001, 0),
                driveSubsystem::setDesiredState,
                events,
                true,
                driveSubsystem);

        Command fullAuto = builder.fullAuto(trajectory);

        return fullAuto;
    }

    public static Command runPath(String name, DriveSubsystem driveSubsystem) {
        var trajectory = PathPlanner.loadPath(name, new PathConstraints(4, 1));
        SwerveAutoBuilder builder = new SwerveAutoBuilder(
                driveSubsystem::getPose,
                driveSubsystem::resetOdometry,
                driveSubsystem.getKinematics(),
                new PIDConstants(0.7, 0.0001, 0.0),
                new PIDConstants(0.1, 0.0001, 0),
                driveSubsystem::setDesiredState,
                new HashMap<String, Command>(),
                true,
                driveSubsystem);

        Command fullAuto = builder.fullAuto(trajectory);

        return fullAuto;
    }

    public static Command placePiece(RobotContainer container) {
        return Commands.sequence(
                new InstantCommand(() -> {
                    GamePieceType currentGamePiece = container.m_preloadedPieceChooser.getSelected();
                    container.m_armGamePieceSubsystem.gamePiecePickup(currentGamePiece);
                }),
                new WaitCommand(0.5),
                new LineupArmCommand(container.m_armRotationSubsystem, container.m_armExtensionSubsystem,
                        container.m_armGamePieceSubsystem, ArmPositioningType.High),
                Commands.either(new ArmReleaseCubeCommand(container.m_armGamePieceSubsystem),
                        new ArmReleaseConeCommand(container.m_armGamePieceSubsystem, container.m_armRotationSubsystem),
                        () -> {
                            return container.m_preloadedPieceChooser.getSelected() == GamePieceType.Cube;
                        }),
                new WaitCommand(0.1),
                new ArmToLoadingCommand(container.m_armRotationSubsystem, container.m_armExtensionSubsystem)
        );
    }

    public static Command placeAndLeaveLeft(RobotContainer container) {
        return Commands.sequence(
                placePiece(container),
                runPath("Place And Leave Left", container.m_driveSubsystem));
    }

    public static Command placeAndLeaveRight(RobotContainer container) {
        return Commands.sequence(
                placePiece(container),
                runPath("Place And Leave Right", container.m_driveSubsystem));
    }

    public static Command placeAndLeaveAndBalance(RobotContainer container) {
        return Commands.sequence(
                placePiece(container),
                runPath("Place And Leave And Balance", container.m_driveSubsystem),
                new AutoBalanceCommand(container.m_driveSubsystem));
    }

    public static Command placeAndBalance(RobotContainer container) {
        return Commands.sequence(
                placePiece(container),
                runPath("Place And Balance", container.m_driveSubsystem),
                new AutoBalanceCommand(container.m_driveSubsystem));
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
