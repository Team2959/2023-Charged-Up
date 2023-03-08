package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.PlacementArmSubsystem;
import frc.robot.subsystems.PlacementArmSubsystem.GamePieceType;

public class ArmExtensionInitalizeCommand extends InstantCommand {

    PlacementArmSubsystem m_PlacementArmSubsystem;

    Supplier<GamePieceType> gamePiece;
    public ArmExtensionInitalizeCommand(PlacementArmSubsystem placementArmSubsystem, Supplier<GamePieceType> gamePieceTypeSupplier) {
        m_PlacementArmSubsystem = placementArmSubsystem;
        gamePiece = gamePieceTypeSupplier;
        // addRequirements(placementArmSubsystem);
    }

    @Override
    public void initialize() {
        m_PlacementArmSubsystem.initalizeArm(gamePiece.get());
        m_PlacementArmSubsystem.setArmExtensionPosition(0);
    }
}
