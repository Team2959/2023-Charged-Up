// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmExtensionSubsystem;

public class ArmExtentionCommand extends CommandBase {
    private double m_targetDistance;
    private ArmExtensionSubsystem m_armExtensionSubsystem;

    public ArmExtentionCommand(ArmExtensionSubsystem armExtensionSubsystem, double distance) {
        addRequirements(armExtensionSubsystem);
        m_armExtensionSubsystem = armExtensionSubsystem;
        m_targetDistance = distance;
    }

    @Override
    public void initialize() {
        m_armExtensionSubsystem.setArmExtensionPosition(m_targetDistance);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted)
            m_armExtensionSubsystem.stopArmExtensionMotor();
    }

    @Override
    public boolean isFinished() {
        return m_armExtensionSubsystem.isArmExtensionAtSetpoint();
    }
}
