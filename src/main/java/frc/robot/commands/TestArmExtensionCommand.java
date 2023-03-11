// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PlacementArmSubsystem;

public class TestArmExtensionCommand extends CommandBase {
    private PlacementArmSubsystem m_placementArmSubsystem;

    /** Creates a new TestArmExtensionCommand. */
    public TestArmExtensionCommand(PlacementArmSubsystem placementArmSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_placementArmSubsystem = placementArmSubsystem;
        addRequirements(placementArmSubsystem);
        SmartDashboard.putNumber("Test Arm Extension Target Position", 0);
    }

    @Override
    public void initialize() {
        double target = SmartDashboard.getNumber("Test Arm Extension Target Position", 0);
        m_placementArmSubsystem.setArmExtensionPosition(target);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return m_placementArmSubsystem.isArmExtensionAtSetpoint();
    }
}
