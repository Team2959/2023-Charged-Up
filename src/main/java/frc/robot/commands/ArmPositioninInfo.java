package frc.robot.commands;

import javax.lang.model.util.ElementScanner14;

import frc.robot.subsystems.PlacementArmSubsystem.GamePieceType;

public final class ArmPositioninInfo {
    public enum ArmPositioningType {
        High,
        Mid,
        Low,
        WallLineup,
        WallPickup,
        WallHorizLineup,
    };

    public static double getArmDistance(ArmPositioningType armPosition, GamePieceType gamePieceType) {
        switch (armPosition) {
            case High:
                return 115;
            case Mid:
                if(gamePieceType == GamePieceType.Cone)
                    return 50;
                else
                    return 10;
            case Low:
                return 30;
            case WallLineup:
            case WallPickup:
                return 60;
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
                    return 165;
                else
                    return 150;
            case Mid:
                if (gamePieceType == GamePieceType.Cone)
                    return 153;
                else
                    return 130;
            case Low:
                return 115;
            case WallLineup:
                return 180;
            case WallPickup:
                return 160;
            case WallHorizLineup:
            if(gamePieceType == GamePieceType.Cone) 
                return 153;
            else
                return 147;
            default:
                return 0;
        }
    }
}
