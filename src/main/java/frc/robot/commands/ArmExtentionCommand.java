// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PlacementArmSubsystem;

public class ArmExtentionCommand extends CommandBase {
    private double m_targetDistance;
    private PlacementArmSubsystem m_placementArmSubsystem;

    public ArmExtentionCommand(PlacementArmSubsystem placementArmSubsystem, double distance) {
        addRequirements(placementArmSubsystem);
        m_placementArmSubsystem = placementArmSubsystem;
        m_targetDistance = distance;
    }

    @Override
    public void initialize() {
        m_placementArmSubsystem.setArmExtensionPosition(m_targetDistance);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        m_placementArmSubsystem.stopArmExtensionMotor();
    }

    @Override
    public boolean isFinished() {
        return m_placementArmSubsystem.isArmExtensionAtSetpoint();
    }
}
