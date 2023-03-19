// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.ArmPositioninInfo.ArmPositioningType;
import frc.robot.subsystems.ArmGamePieceControlSubsystem;
import frc.robot.subsystems.ArmRotationSubsystem;

public class ArmRotationByGamePieceAndPositionCommand extends CommandBase {
    private ArmRotationSubsystem m_armRotationSubsystem;
    private ArmGamePieceControlSubsystem m_armGamePieceControlSubsystem;
    private ArmPositioningType m_positioningType;

    public ArmRotationByGamePieceAndPositionCommand(ArmRotationSubsystem armRotationSubsystem,
        ArmGamePieceControlSubsystem armGamePieceControlSubsystem,
        ArmPositioningType positioningType)
    {
        m_armRotationSubsystem = armRotationSubsystem;
        m_armGamePieceControlSubsystem = armGamePieceControlSubsystem;
        m_positioningType = positioningType;
        addRequirements(armRotationSubsystem);
    }

    @Override
    public void initialize() {
        m_armRotationSubsystem.setArmDegrees(
            ArmPositioninInfo.getArmAngle(m_positioningType,
                m_armGamePieceControlSubsystem.getGamePieceType(),
                m_armGamePieceControlSubsystem.getUnloadType()));
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted)
            m_armRotationSubsystem.stopArmRotatorMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_armRotationSubsystem.isArmRotatorAtSetpoint();
    }
}
