package frc.robot.commands;

import frc.robot.subsystems.ArmGamePieceControlSubsystem.GamePieceType;

public final class ArmPositioninInfo {
    public enum ArmPositioningType {
        High,
        Mid,
        Low,
        WallHorizLineup,
    };

    public static double getArmDistance(ArmPositioningType armPosition, GamePieceType gamePieceType) {
        switch (armPosition) {
            case High:
                return 90;
            case Mid:
                if(gamePieceType == GamePieceType.Cone)
                    return 10;
                else
                    return 0;
            case Low:
                return 0;
            case WallHorizLineup:
                return 0;
            default:
                return 0;
        }
    }

    public static double getArmAngle(ArmPositioningType armPosition, GamePieceType gamePieceType) {
        switch (armPosition) {
            case High:
                if (gamePieceType == GamePieceType.Cone)
                    return 160;
                else
                    return 152;
            case Mid:
                if (gamePieceType == GamePieceType.Cone)
                    return 143;
                else
                    return 130;
            case Low:
                return 100;
            case WallHorizLineup:
                return 147;
            default:
                return 0;
        }
    }
}
