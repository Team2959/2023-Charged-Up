// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PlacementArmSubsystem;

public class ArmRotationCommand extends CommandBase {
    private PlacementArmSubsystem m_placementArmSubsystem;
    private double m_angle;

    public ArmRotationCommand(PlacementArmSubsystem placementArmSubsystem, double angleInDegrees) {
        m_placementArmSubsystem = placementArmSubsystem;
        m_angle = angleInDegrees;

        addRequirements(placementArmSubsystem);
    }

    @Override
    public void initialize() {
        m_placementArmSubsystem.setArmDegrees(m_angle);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        m_placementArmSubsystem.stopArmRotatorMotor();
    }

    @Override
    public boolean isFinished() {
        return m_placementArmSubsystem.isArmRotatorAtSetpoint();
    }
}
