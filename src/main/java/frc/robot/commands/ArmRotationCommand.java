// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmRotationSubsystem;

public class ArmRotationCommand extends CommandBase {
    private ArmRotationSubsystem m_ArmRotationSubsystem;
    private double m_angle;

    public ArmRotationCommand(ArmRotationSubsystem armRotationSubsystem, double angleInDegrees) {
        m_ArmRotationSubsystem = armRotationSubsystem;
        m_angle = angleInDegrees;

        addRequirements(armRotationSubsystem);
    }

    @Override
    public void initialize() {
        m_ArmRotationSubsystem.setArmDegrees(m_angle);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted)
        m_ArmRotationSubsystem.stopArmRotatorMotor();
    }

    @Override
    public boolean isFinished() {
        return m_ArmRotationSubsystem.isArmRotatorAtSetpoint();
    }
}
