package frc.robot.commands;

import frc.robot.subsystems.ArmGamePieceControlSubsystem.GamePieceType;
import frc.robot.subsystems.ArmGamePieceControlSubsystem.UnloadType;

public final class ArmPositioninInfo {
    public enum ArmPositioningType {
        High,
        Mid,
        Low,
        WallHorizLineup,
        FloorPickup,
        SlidingWallPickup,
    };

    public static double getArmDistance(
        ArmPositioningType armPosition,
        GamePieceType gamePieceType,
        UnloadType unloadSide)
    {
        switch (armPosition) {
            case High:
                // return unloadSide == UnloadType.Front ? 90 : 110;
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

    public static double getArmAngle(
        ArmPositioningType armPosition,
        GamePieceType gamePieceType,
        UnloadType unloadSide)
    {
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
                if(gamePieceType == GamePieceType.Cone)
                    return 147;
                else
                    return 150;
            default:
                return 0;
        }
    }
}
