// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.PlacementArmSubsystem;

public class ArmVacuumReleaseCommand extends SequentialCommandGroup {
    private PlacementArmSubsystem m_placementArmSubsystem;

    // TODO: handle this command being interrupted somehow
    public ArmVacuumReleaseCommand(PlacementArmSubsystem placementArmSubsystem) {
        m_placementArmSubsystem = placementArmSubsystem;
        addCommands(
            new InstantCommand(() -> m_placementArmSubsystem.manipulateVacuumRelease(true)),
            new WaitCommand(1),
            new InstantCommand(() -> m_placementArmSubsystem.manipulateVacuumRelease(false)));
    }

}
