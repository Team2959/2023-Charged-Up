// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TeleOpDriveCommand extends CommandBase {
  private DriveSubsystem m_driveSubsystem;
  private Joystick m_leftJoystick;
  private Joystick m_rightJoystick;

  /** Creates a new TeleOpDriveCommand. */
  public TeleOpDriveCommand(DriveSubsystem driveSubsystem, Joystick leftJoystick, Joystick rightJoystick)
  {
    m_driveSubsystem = driveSubsystem;
    m_leftJoystick = leftJoystick;
    m_rightJoystick = rightJoystick;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    m_driveSubsystem.drive(m_rightJoystick.getX(), m_rightJoystick.getY(), m_leftJoystick.getX(), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
