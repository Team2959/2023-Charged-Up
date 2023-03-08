// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.ArmPositioninInfo.ArmPositioningType;
import frc.robot.subsystems.PlacementArmSubsystem;

public class ArmRotationByArmPositionTypeCommand extends CommandBase {
  private PlacementArmSubsystem m_placementArmSubsystem;
  private ArmPositioningType m_positioningType;

  /** Creates a new ArmRotationGamePieceDeliverCommand. */
  public ArmRotationByArmPositionTypeCommand(PlacementArmSubsystem placementArmSubsystem, ArmPositioningType positioningType) {
    m_placementArmSubsystem = placementArmSubsystem;
    m_positioningType = positioningType;
    addRequirements(placementArmSubsystem);
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_placementArmSubsystem.setArmDegrees(
      ArmPositioninInfo.getArmAngle(m_positioningType, m_placementArmSubsystem.getGamePieceType()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted)
      m_placementArmSubsystem.stopArmRotatorMotor();
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_placementArmSubsystem.isArmRotatorAtSetpoint();
  }
}
