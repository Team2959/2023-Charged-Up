package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;

public class AprilTagNavCommand extends CommandBase {
    Vision m_vision;
    DriveSubsystem m_driveSubsystem;

    public AprilTagNavCommand(Vision vision, DriveSubsystem driveSubsystem) {
        m_vision = vision;
        m_driveSubsystem = driveSubsystem;
    }

    @Override
    public void initialize() {
        Pose2d currentLoc = m_vision.getBotPose().toPose2d(m_driveSubsystem.getAngle());
        var loc = m_vision.getNav().nearestLocationForTeam(currentLoc, DriverStation.getAlliance() == Alliance.Red);
        
        SmartDashboard.putNumber("AprilNav/Current/X", currentLoc.getX());
        SmartDashboard.putNumber("AprilNav/Current/Y", currentLoc.getY());
        SmartDashboard.putNumber("AprilNav/Current/Rotation", currentLoc.getRotation().getDegrees());

        SmartDashboard.putString("AprilNav/Destination/Name", loc.getKey());
        SmartDashboard.putNumber("AprilNav/Destination/X", loc.getValue().getX());
        SmartDashboard.putNumber("AprilNav/Destination/Y", loc.getValue().getY());
        SmartDashboard.putNumber("AprilNav/Destination/Rotation", loc.getValue().getRotation().getDegrees());


    }
}
