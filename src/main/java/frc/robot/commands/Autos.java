// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto() {
    return null;
  }

  public static Command runPath(DriveSubsystem driveSubsystem) {
    var trajectory = PathPlanner.loadPath("Basic", new PathConstraints(4, 1));
    SwerveAutoBuilder builder = new SwerveAutoBuilder(
        driveSubsystem::getPose,
        driveSubsystem::resetOdometry,
        driveSubsystem.getKinematics(),
        new PIDConstants(0.7,  0.0001, 0.0),
        new PIDConstants(0.1, 0.0001, 0),
        driveSubsystem::setDesiredState,
        new HashMap<String, Command>(),
        true,
        driveSubsystem
    );

    Command fullAuto = builder.fullAuto(trajectory);

    return fullAuto;
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
