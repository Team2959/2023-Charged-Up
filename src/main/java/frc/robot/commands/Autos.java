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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmPositioninInfo.ArmPositioningType;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmGamePieceControlSubsystem;
import frc.robot.subsystems.ArmRotationSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmGamePieceControlSubsystem.GamePieceType;

public final class Autos {
    /** Example static factory for an autonomous command. */
    public static CommandBase exampleAuto() {
        return null;
    }

    public static SendableChooser<Command> sendableChooser(RobotContainer container) {
        SendableChooser<Command> sendableChooser = new SendableChooser<>();
        sendableChooser.addOption("Nothing", new WaitCommand(0));
        // sendableChooser.addOption("Test Bump", new
        // BumpCubeCommand(container.m_PlacementArmSubsystem));
        sendableChooser.addOption("Basic Cube Not Over Charging Station",
                (new BumpCubeCommand(container.m_ArmRotationSubsystem, container.m_ArmExtensionSubsystem)
                        .andThen(runPath("Basic Cube", container.m_driveSubsystem))));
        sendableChooser.addOption("Basic Cube Over Charging Station",
                (new BumpCubeCommand(container.m_ArmRotationSubsystem, container.m_ArmExtensionSubsystem)
                        .andThen(runPath("Basic Cube", container.m_driveSubsystem))));
        sendableChooser.setDefaultOption("Place Cone Auto",
                placeGamePiece(GamePieceType.Cone, container.m_driveSubsystem,
                        container.m_ArmRotationSubsystem,
                        container.m_ArmExtensionSubsystem,
                        container.m_ArmGamePieceSubsystem,
                         container.m_IntakeSubsystem));
        sendableChooser.addOption("Place Cone and Balance NOT TESTED", placeGamePieceAndBalance(
            GamePieceType.Cone, container.m_driveSubsystem,
            container.m_ArmRotationSubsystem,
            container.m_ArmExtensionSubsystem,
            container.m_ArmGamePieceSubsystem));
        sendableChooser.addOption("Drive Only", runPath("Place Game Piece", container.m_driveSubsystem));
        // sendableChooser.addOption("Basic Cube 1",
        // (new CubeExtractionCommandGroup(container.m_IntakeSubsystem,
        // container.m_PlacementArmSubsystem)
        // .andThen(runPath("Basic Cube", container.m_driveSubsystem))));
        // sendableChooser.addOption("Basic Cube 2",
        // (new CubeExtractionCommandGroup(container.m_IntakeSubsystem,
        // container.m_PlacementArmSubsystem)
        // .andThen(runPath("Basic Cube 2", container.m_driveSubsystem))));
        // sendableChooser.addOption("Basic Cube 3",
        // (new CubeExtractionCommandGroup(container.m_IntakeSubsystem,
        // container.m_PlacementArmSubsystem)
        // .andThen(runPath("Basic Cube 3", container.m_driveSubsystem))));
        // sendableChooser.addOption("Basic Cube 4",
        // (new CubeExtractionCommandGroup(container.m_IntakeSubsystem,
        // container.m_PlacementArmSubsystem)
        // .andThen(runPath("Basic Cube 4", container.m_driveSubsystem))));
        return sendableChooser;
    }

    public static Command placeGamePiece(GamePieceType gamePieceType, DriveSubsystem driveSubsystem,
            ArmRotationSubsystem armRotationSubsystem,
            ArmExtensionSubsystem armExtensionSubsystem,
            ArmGamePieceControlSubsystem armGamePieceControlSubsystem,
            IntakeSubsystem intakeSubsystem) {
        Command readyPiece = Commands.sequence(
                new InstantCommand(() -> { armGamePieceControlSubsystem.gamePiecePickup(gamePieceType); }),
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
            new InstantCommand(() -> { armGamePieceControlSubsystem.gamePiecePickup(gamePieceType); }),
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
                .andThen(new ArmToLoadingCommand(armRotationSubsystem, armExtensionSubsystem)).andThen(new AutoBalanceCommand(driveSubsystem));
    }

    public static Command runPath(String name, HashMap<String, Command> events, DriveSubsystem driveSubsystem) {
        var trajectory = PathPlanner.loadPath(name, new PathConstraints(4, 1));
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

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
