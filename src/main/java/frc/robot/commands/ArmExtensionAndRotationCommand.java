package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmRotationSubsystem;

public class ArmExtensionAndRotationCommand extends SequentialCommandGroup {

    static final double kFrontDegrees = 80;
    static final double kBackDegrees = 20;

    public ArmExtensionAndRotationCommand(ArmExtensionSubsystem armExtensionSubsystem, ArmRotationSubsystem armRotationSubsystem, double extension, double rotation) {
    
        addCommands(
            Commands.either(
                new ArmExtentionCommand(armExtensionSubsystem, 0),
                Commands.none(),
                () -> {
                if(rotation - armRotationSubsystem.lastArmRotationTarget() > 0) {
                    if(armRotationSubsystem.lastArmRotationTarget() < kFrontDegrees) {
                        return true;
                    }
                }
                else {
                    if(armRotationSubsystem.lastArmRotationTarget() > kBackDegrees) {
                        return true;
                    }
                }
                
                return false;
            })
        );
    }

}
