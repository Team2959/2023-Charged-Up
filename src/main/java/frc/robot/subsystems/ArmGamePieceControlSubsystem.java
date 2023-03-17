// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import cwtech.util.SolenoidV2;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ArmGamePieceControlSubsystem extends SubsystemBase {
    public enum GamePieceType {
        Unknown,
        Cone,
        Cube
    };

    public enum UnloadType {
        Front,
        Back,
    };


    private UnloadType m_unloadType = UnloadType.Front;
    private GamePieceType m_GamePieceType = GamePieceType.Unknown;

    private Spark m_gripVacuumMotor = new Spark(RobotMap.kGripVacuumSparkMotor);
    private SolenoidV2 m_armVacuumRelease1 = new SolenoidV2(RobotMap.kArmVacuumRelease1);
    private SolenoidV2 m_armVacuumRelease2 = new SolenoidV2(RobotMap.kArmVacuumRelease2);
    private SolenoidV2 m_armVacuumRelease3 = new SolenoidV2(RobotMap.kArmVacuumRelease3);
    private SolenoidV2 m_armVacuumRelease4 = new SolenoidV2(RobotMap.kArmVacuumRelease4);

    /** Creates a new ArmGamePieceControlSubsystem. */
    public ArmGamePieceControlSubsystem() {
    }

    public void setUnloadType(UnloadType unloadType) {
        m_unloadType = unloadType;
    }

    public UnloadType getUnloadType() {
        return m_unloadType;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void gamePiecePickup(GamePieceType gamePieceType) {
        manipulateVacuumMotors(true);
        manipulateVacuumSolenoids(false);
        setGamePieceType(gamePieceType);
    }

    public void gamePieceRelease() {
        manipulateVacuumMotors(false);
        manipulateVacuumSolenoids(true);
        setGamePieceType(GamePieceType.Unknown);
    }

    public void resetVacuumSoloenoids() {
        manipulateVacuumSolenoids(false);
    }

    private void manipulateVacuumSolenoids(boolean releaseVacuum) {
        m_armVacuumRelease1.set(releaseVacuum);
        m_armVacuumRelease2.set(releaseVacuum);
        m_armVacuumRelease3.set(releaseVacuum);
        m_armVacuumRelease4.set(releaseVacuum);
    }

    private void manipulateVacuumMotors(boolean evacuate) {
        var speed = evacuate ? 1.0 : 0.0;
        m_gripVacuumMotor.set(speed);
    }

    private void setGamePieceType(GamePieceType gamePiece) {
        m_GamePieceType = gamePiece;
    }

    public GamePieceType getGamePieceType() {
        return m_GamePieceType;
    }
}
