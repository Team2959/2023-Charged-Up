// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmExtensionSubsystem;

public class TestArmExtensionCommand extends CommandBase {
    private ArmExtensionSubsystem m_armExtensionSubsystem;

    /** Creates a new TestArmExtensionCommand. */
    public TestArmExtensionCommand(ArmExtensionSubsystem armExtensionSubsystem) {
        m_armExtensionSubsystem = armExtensionSubsystem;
        addRequirements(armExtensionSubsystem);
        SmartDashboard.putNumber("Test Arm Extension Target Position", 0);
    }

    @Override
    public void initialize() {
        double target = SmartDashboard.getNumber("Test Arm Extension Target Position", 0);
        m_armExtensionSubsystem.setArmExtensionPosition(target);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return m_armExtensionSubsystem.isArmExtensionAtSetpoint();
    }
}
