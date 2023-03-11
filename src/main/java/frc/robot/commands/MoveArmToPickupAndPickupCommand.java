package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PlacementArmSubsystem;

public class MoveArmToPickupAndPickupCommand extends SequentialCommandGroup {
    PlacementArmSubsystem m_placementArmSubsystem;
    IntakeSubsystem m_intakeSubsystem;

    MoveArmToPickupAndPickupCommand(PlacementArmSubsystem placementArmSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(
                new ArmExtentionCommand(m_placementArmSubsystem, 0),
                new ArmRotationCommand(m_placementArmSubsystem, 0), // TODO actual values for this command
                new ArmExtentionCommand(m_placementArmSubsystem, 10),
                new ArmEngageVacuumCommand(m_placementArmSubsystem));
    }

}
