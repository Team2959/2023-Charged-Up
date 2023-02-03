package frc.robot.commands;

import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DriveSubsystem;

public class RunPathCommand extends SequentialCommandGroup {

    DriveSubsystem m_driveSubsystem;
    Command m_pathCommand;
    Pose2d m_pose;
    int m_ticks = 0;

    public Rotation2d desiredRotation() {
        m_ticks += 1;
        return Rotation2d.fromDegrees(90);
    }

    public RunPathCommand(DriveSubsystem driveSubsystem, String path) {
        try {
            m_driveSubsystem = driveSubsystem;
            Path path2 = Filesystem.getDeployDirectory().toPath().resolve("pathweaver/output/" + path + ".wpilib.json");
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(path2);
            SwerveControllerCommand command = new SwerveControllerCommand(trajectory,
                    () -> m_driveSubsystem.getPose(),
                    m_driveSubsystem.getKinematics(),
                    new PIDController(0.1,  0.0001, 0.0),
                    new PIDController(0.2, 0.0001, 0),
                    new ProfiledPIDController(0.1, 0, 0, new Constraints(Math.PI, Math.PI)),
                    // () -> desiredRotation(),
                    (SwerveModuleState[] states) -> m_driveSubsystem.setDesiredState(states),
                    m_driveSubsystem);
            m_pathCommand = command;
            m_pose = trajectory.getInitialPose();
            addCommands(
                new InstantCommand(() -> {
                    startup();
                }),
                m_pathCommand,
                new InstantCommand(() -> {
                    cleanup();
                })
            );
        }
        catch(Exception e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
        }
    }

    public void startup() {
        m_pose = new Pose2d(new Translation2d(m_pose.getX(), m_pose.getY()), new Rotation2d(0));
        m_driveSubsystem.resetOdometry(m_pose);
    }

    public void cleanup() {
        m_driveSubsystem.drive(0, 0, 0, false);
    }
}

