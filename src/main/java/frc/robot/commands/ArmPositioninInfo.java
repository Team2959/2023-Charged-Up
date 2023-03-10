package frc.robot.commands;

import frc.robot.subsystems.PlacementArmSubsystem.GamePieceType;

public final class ArmPositioninInfo {
    public enum ArmPositioningType
    {
      High,
      Mid,
      Low,
      WallLineup,
      WallPickup,
      WallHorizLineup,
    };
  
    public static double getArmDistance(ArmPositioningType armPosition)
    {
        switch (armPosition)
        {
          case High:
            return 115; 
          case Mid:
            return 50;
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
  
    public static double getArmAngle(ArmPositioningType armPosition, GamePieceType gamePieceType)
    {
        switch (armPosition)
        {
          case High:
            if (gamePieceType == GamePieceType.Cone)
              return 185;
            else
              return 180;
          case Mid:
            if (gamePieceType == GamePieceType.Cone)
              return 170;
            else
              return 155;
          case Low:
            return 115;
          case WallLineup:
            return 180;
          case WallPickup:
            return 160;
          case WallHorizLineup:
            return 157;
        default:
            return 0;
        }
    }
  }
