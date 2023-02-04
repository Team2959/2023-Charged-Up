package frc.robot.commands;

import java.nio.file.Path;
import java.util.HashMap;

import javax.swing.SpringLayout.Constraints;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class RunPathCommand2 extends CommandBase {
    PathPlannerTrajectory m_trajectory;
    public RunPathCommand2(DriveSubsystem driveSubsystem) {
        m_trajectory = PathPlanner.loadPath("Basic", new PathConstraints(4, 1));
        SwerveAutoBuilder builder = new SwerveAutoBuilder(
            driveSubsystem::getPose,
            driveSubsystem::resetOdometry,
            driveSubsystem.getKinematics(),
            new PIDConstants(0.1,  0.0001, 0.0),
                    new PIDConstants(0.2, 0.0001, 0),
            driveSubsystem::setDesiredState,
            new HashMap<String, Command>(),
            true,
            driveSubsystem
        );

        Command fullAuto = builder.fullAuto(m_trajectory);
    }
}
