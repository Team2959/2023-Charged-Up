// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmRotationSubsystem;

public class TestArmRotationCommand extends CommandBase {
    private ArmRotationSubsystem m_armRotationSubsystem;

    public TestArmRotationCommand(ArmRotationSubsystem armRotationSubsystem) {
        m_armRotationSubsystem = armRotationSubsystem;
        addRequirements(armRotationSubsystem);
        SmartDashboard.putNumber("Test Arm Rotation Target Position", ArmRotationSubsystem.kArmHomePosition);
    }

    @Override
    public void initialize() {
        double target = SmartDashboard.getNumber("Test Arm Rotation Target Position",
            ArmRotationSubsystem.kArmHomePosition);
        m_armRotationSubsystem.setArmDegrees(target);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return m_armRotationSubsystem.isArmRotatorAtSetpoint();
    }
}
