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
import frc.robot.RobotContainer;
import frc.robot.commands.ArmPositioninInfo.ArmPositioningType;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ArmGamePieceControlSubsystem.GamePieceType;

public final class Autos {
    private static final double kRestractExtensionAtStartUpPosition = 0;

    public static SendableChooser<Command> sendableChooser(RobotContainer container) {
        SendableChooser<Command> sendableChooser = new SendableChooser<>();
        sendableChooser.addOption("Nothing", new WaitCommand(0));
        sendableChooser.setDefaultOption("Place And Leave Left", placeAndLeaveLeft(container));
        sendableChooser.addOption("Place And Leave Right", placeAndLeaveRight(container));
        sendableChooser.addOption("Place And Leave And Balance", placeAndLeaveAndBalance(container));
        sendableChooser.addOption("Place And Balance", placeAndBalance(container));
        sendableChooser.addOption("Place Mid And Balance", placeMidAndBalance(container));
        sendableChooser.addOption("Drive Only", runPath("Place Game Piece", container.m_driveSubsystem));
        sendableChooser.addOption("Place And Leave And Locate Cone", placeAndLeaveAndLocateCone(container));
        return sendableChooser;
    }

    // public static Command placeGamePiece(GamePieceType gamePieceType,
    // DriveSubsystem driveSubsystem,
    // ArmRotationSubsystem armRotationSubsystem,
    // ArmExtensionSubsystem armExtensionSubsystem,
    // ArmGamePieceControlSubsystem armGamePieceControlSubsystem,
    // IntakeSubsystem intakeSubsystem) {
    // Command readyPiece = Commands.sequence(
    // new InstantCommand(() -> {
    // armGamePieceControlSubsystem.gamePiecePickup(gamePieceType);
    // }),
    // new WaitCommand(0.5), // remove?
    // new ArmExtentionCommand(armExtensionSubsystem,
    // kRestractExtensionAtStartUpPosition),
    // new LineupArmCommand(armRotationSubsystem, armExtensionSubsystem,
    // armGamePieceControlSubsystem,
    // ArmPositioningType.High),
    // new WaitCommand(0.5), // why wait? or reduce
    // new ArmVacuumReleaseCommand(armGamePieceControlSubsystem),
    // new WaitCommand(0.5)); // see if can reduce!! 0.5?
    // return readyPiece.andThen(runPath("Place Game Piece", driveSubsystem))
    // .alongWith(new ArmToLoadingCommand(armRotationSubsystem,
    // armExtensionSubsystem));
    // }

    // public static Command placeGamePieceAndBalance(GamePieceType gamePieceType,
    // DriveSubsystem driveSubsystem,
    // ArmRotationSubsystem armRotationSubsystem,
    // ArmExtensionSubsystem armExtensionSubsystem,
    // ArmGamePieceControlSubsystem armGamePieceControlSubsystem) {
    // Command readyPiece = Commands.sequence(
    // new InstantCommand(() -> {
    // armGamePieceControlSubsystem.gamePiecePickup(gamePieceType);
    // }),
    // new WaitCommand(0.5), // remove?
    // new ArmExtentionCommand(armExtensionSubsystem,
    // kRestractExtensionAtStartUpPosition),
    // new LineupArmCommand(armRotationSubsystem, armExtensionSubsystem,
    // armGamePieceControlSubsystem,
    // ArmPositioningType.High),
    // new WaitCommand(0.5), // why wait?
    // new ArmVacuumReleaseCommand(armGamePieceControlSubsystem),
    // new WaitCommand(0.5)); // remove/reduce?
    // return readyPiece.andThen(runPath("Place Game Piece And Balance",
    // driveSubsystem))
    // .alongWith(new ArmToLoadingCommand(armRotationSubsystem,
    // armExtensionSubsystem))
    // .andThen(new AutoBalanceCommand(driveSubsystem));
    // }

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
        var trajectory = PathPlanner.loadPath(name, new PathConstraints(4, 2));
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
        return placePiece(container, true);
    }

    public static Command placePiece(RobotContainer container, boolean retractToo) {
        return placePiece(container, retractToo, ArmPositioningType.High);
    }

    public static Command placePiece(RobotContainer container, boolean retractToo, ArmPositioningType armPosition) {
        return Commands.sequence(
                new InstantCommand(() -> {
                    var currentGamePiece = container.m_preloadedPieceChooser.getSelected();
                    container.m_armGamePieceSubsystem.gamePiecePickup(currentGamePiece);
                }),
                new WaitCommand(1),
                new ArmExtentionCommand(container.m_armExtensionSubsystem, kRestractExtensionAtStartUpPosition),
                new LineupArmCommand(container.m_armRotationSubsystem, container.m_armExtensionSubsystem,
                        container.m_armGamePieceSubsystem, armPosition),
                Commands.either(new ArmVacuumReleaseCommand(container.m_armGamePieceSubsystem),
                        new ArmReleaseConeCommand(container.m_armGamePieceSubsystem, container.m_armRotationSubsystem),
                        () -> {
                            return container.m_preloadedPieceChooser.getSelected() == GamePieceType.Cube;
                        }),
                // new WaitCommand(0.1),
                retractToo ? new ArmToLoadingCommand(container.m_armRotationSubsystem, container.m_armExtensionSubsystem)
                        : Commands.none());
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

    public static Command placeMidAndBalance(RobotContainer container) {
        return Commands.sequence(
            placePiece(container, false, ArmPositioningType.Mid),
            Commands.sequence(
                runPath("Place And Balance", container.m_driveSubsystem),
                new InstantCommand(() -> container.m_driveSubsystem.stopAndLockWheels()),
                new AutoBalanceCommand(container.m_driveSubsystem)
            ).alongWith(new ArmToLoadingCommand(container.m_armRotationSubsystem, container.m_armExtensionSubsystem))
        );
    }

    public static Command placeAndLeaveAndBalance(RobotContainer container) {
        return Commands.sequence(
                placePiece(container),
                runPath("Place And Leave And Balance", container.m_driveSubsystem),
                new AutoBalanceCommand(container.m_driveSubsystem));
    }

    public static Command placeAndBalance(RobotContainer container) {
        return Commands.sequence(
                placePiece(container, false),
                Commands.sequence(runPath("Place And Balance", container.m_driveSubsystem),
                        new AutoBalanceCommand(container.m_driveSubsystem))
                .alongWith(
                        new ArmToLoadingCommand(container.m_armRotationSubsystem, container.m_armExtensionSubsystem)));
    }

    // NOTE: only works with hardcoded cone value right now
    public static Command placeAndLeaveAndLocateCone(RobotContainer container) {
        return Commands.sequence(
                placePiece(container),
                runPath("Place And Leave and Locate Piece", container.m_driveSubsystem),
                new InstantCommand(() -> container.m_armGamePieceSubsystem.gamePiecePickup(GamePieceType.Cone)),
                new LineupArmCommand(container.m_armRotationSubsystem, container.m_armExtensionSubsystem,
                        container.m_armGamePieceSubsystem, ArmPositioningType.FloorPickup));
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
