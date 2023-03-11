// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeVacuumReleaseCommand extends SequentialCommandGroup {
    public IntakeVacuumReleaseCommand(IntakeSubsystem intakeSubsystem) {
        addCommands(
                new InstantCommand(() -> intakeSubsystem.releaseVacuumSeal()),
                new InstantCommand(() -> intakeSubsystem.stopVacuum()),
                new WaitCommand(1),
                new InstantCommand(() -> intakeSubsystem.engageVacuumSeal()));
    }
}
