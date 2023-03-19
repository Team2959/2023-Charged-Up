// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.ArmPositioninInfo.ArmPositioningType;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmGamePieceControlSubsystem;

public class ArmExtensionByGamePieceAndPositionCommand extends CommandBase {
  private ArmExtensionSubsystem m_armExtensionSubsystem;
  private ArmGamePieceControlSubsystem m_armGamePieceControlSubsystem;
  private ArmPositioningType m_positioningType;

  /** Creates a new ArmExtensionByGamePieceTypeCommand. */
  public ArmExtensionByGamePieceAndPositionCommand(ArmExtensionSubsystem armExtensionSubsystem,
    ArmGamePieceControlSubsystem armGamePieceControlSubsystem,
    ArmPositioningType positioningType)
  {
    m_armExtensionSubsystem = armExtensionSubsystem;
    m_armGamePieceControlSubsystem = armGamePieceControlSubsystem;
    m_positioningType = positioningType;
    addRequirements(armExtensionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armExtensionSubsystem.setArmExtensionPosition(
      ArmPositioninInfo.getArmDistance(m_positioningType,
        m_armGamePieceControlSubsystem.getGamePieceType(),
        m_armGamePieceControlSubsystem.getUnloadType()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted)
      m_armExtensionSubsystem.stopArmExtensionMotor();
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_armExtensionSubsystem.isArmExtensionAtSetpoint();
  }
}
