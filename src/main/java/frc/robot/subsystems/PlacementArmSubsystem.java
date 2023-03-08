// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import cwtech.util.SolenoidV2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class PlacementArmSubsystem extends SubsystemBase {

    public static final double kArmHomePosition = 60;
    private static final double kArmRotatorP = 0.0175;
    private static final double kArmRotatorI = 0;
    private static final double kArmRotatorD = 0;

    private static final double kArmExtensionP = 0.02;
    private static final double kArmExtensionI = 0;
    private static final double kArmExtensionD = 0;
    private static final double kArmExtensionFF = 0;
    private static final double kArmExtensionIzone = 0;
    private static final double kLoadedConeExtensionPositionOffset = 35;
    private static final double kLoadedCubeExtensionPositionOffset = 0;

    public enum GamePieceType {
        Unknown,
        Cone,
        Cube
    };

    private GamePieceType m_GamePieceType = GamePieceType.Unknown;

    private CANSparkMax m_armRotatorMotor = new CANSparkMax(RobotMap.kArmRotatorSparkMaxMotor, MotorType.kBrushless);
    private CANSparkMax m_armExtensionMotor = new CANSparkMax(RobotMap.kArmExtensionSparkMaxMotor, MotorType.kBrushless);
    private DigitalInput m_armRotatorEncoderDigitalInput = new DigitalInput(RobotMap.kRotatorArmEncoderPulseWidthDIO);
    private DutyCycle m_armRotatorEncoderDutyCycle = new DutyCycle(m_armRotatorEncoderDigitalInput);
    private Spark m_gripVacuumMotor = new Spark(RobotMap.kGripVacuumSparkMotor);
    private SolenoidV2 m_armVacuumRelease1 = new SolenoidV2(RobotMap.kArmVacuumRelease1);
    private SolenoidV2 m_armVacuumRelease2 = new SolenoidV2(RobotMap.kArmVacuumRelease2);
    private SolenoidV2 m_armVacuumRelease3 = new SolenoidV2(RobotMap.kArmVacuumRelease3);
    private SolenoidV2 m_armVacuumRelease4 = new SolenoidV2(RobotMap.kArmVacuumRelease4);

    // RopeSensor m_ropeSensor = new RopeSensor(RobotMap.kRopeEncoderDigIO);
    private SparkMaxRelativeEncoder m_extensionEncoder;
    private SparkMaxPIDController m_armExtensionMotorPidController;
    private double m_lastArmExtensionTarget = 0; // Arm starts fully retracted
    private double m_lastArmRotationTarget = kArmHomePosition;

    private PIDController m_armRotatorMotorPidController = new PIDController(kArmRotatorP, kArmRotatorI, kArmRotatorD);

    private static double kDt = 0.02;
    private static double kTravelVelocity = 50;
    private TrapezoidProfile.Constraints kConstraints = new Constraints(kTravelVelocity, 50);
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile m_armRotatorTrapezoidProfile = null;
    private SparkMaxPIDController m_rotationPIDController;
    private SparkMaxRelativeEncoder m_rotationMotorEncoder;

    // ** Creates a new PlacementArmSubsystem. */
    public PlacementArmSubsystem() {
        m_armExtensionMotor.restoreFactoryDefaults();

        m_armRotatorMotor.setIdleMode(IdleMode.kBrake);
        m_armExtensionMotor.setIdleMode(IdleMode.kBrake);

        m_rotationMotorEncoder = (SparkMaxRelativeEncoder) m_armRotatorMotor.getEncoder();
        m_rotationPIDController = m_armRotatorMotor.getPIDController();
        // m_rotationPIDController.setP(kArmRotatorP);
        // m_rotationPIDController.setI(kArmRotatorI);
        // m_rotationPIDController.setD(kArmRotatorD);

        m_armRotatorMotorPidController.setP(kArmRotatorP);
        m_armRotatorMotorPidController.setI(kArmRotatorI);
        m_armRotatorMotorPidController.setD(kArmRotatorD);

        m_extensionEncoder = (SparkMaxRelativeEncoder) m_armExtensionMotor.getEncoder();
        m_armExtensionMotorPidController = m_armExtensionMotor.getPIDController();

        m_armExtensionMotorPidController.setP(kArmExtensionP);
        m_armExtensionMotorPidController.setI(kArmExtensionI);
        m_armExtensionMotorPidController.setD(kArmExtensionD);
        m_armExtensionMotorPidController.setFF(kArmExtensionFF);
        m_armExtensionMotorPidController.setIZone(kArmExtensionIzone);

        m_setpoint = new TrapezoidProfile.State(getArmAngle(), 0);
        setArmDegrees(kArmHomePosition);
        
        m_extensionEncoder.setPosition(0);
        setArmExtensionPosition(getArmExtensionPosition());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        SmartDashboard.putNumber(getName() + "/Arm Rotation Encoder Position", getArmAngle());
        SmartDashboard.putNumber(getName() + "/Arm Extension Ticks", getArmExtensionPosition());
        
        double raw =  m_armRotatorMotorPidController.calculate(getArmAngle());
        // SmartDashboard.putNumber(getName() + "/Arm Rotator Raw Output", raw);
        m_armRotatorMotor.set(raw);

        // If switch back to rope encoder
        // m_armExtensionMotor.set(m_armExtensionMotorPidController.calculate(getArmExtensionPosition()));      

        // Trapezoidal profile testing
        m_setpoint = m_armRotatorTrapezoidProfile.calculate(kDt);
        SmartDashboard.putNumber(getName() + "/Arm Rotation Trap Target Position", m_setpoint.position);
        SmartDashboard.putNumber(getName() + "/Arm Rotation Trap Target Velocity", m_setpoint.velocity);
        SmartDashboard.putNumber(getName() + "/Arm Rotation Motor Velocity", m_rotationMotorEncoder.getVelocity());
        // m_rotationPIDController.setReference(m_setpoint.velocity, ControlType.kVelocity);
        // m_rotationPIDController.setReference(m_setpoint.velocity, ControlType.kSmartVelocity);
    }

    public void setupRopeSensor(Robot robot) {
        // m_ropeSensor.setup(robot);
    }

    public void smartDashboardInit() {
        SmartDashboard.putNumber(getName() + "/Arm Rotation Trap Target Position", 0);
        SmartDashboard.putNumber(getName() + "/Arm Rotation Trap Target Velocity", 0);
        SmartDashboard.putNumber(getName() + "/Arm Rotator P", m_armExtensionMotorPidController.getP());
        SmartDashboard.putNumber(getName() + "/Arm Rotator I", m_armExtensionMotorPidController.getI());
        SmartDashboard.putNumber(getName() + "/Arm Rotator D", m_armExtensionMotorPidController.getD());

        SmartDashboard.putNumber(getName() + "/Arm Extension P", m_armExtensionMotorPidController.getP());
        SmartDashboard.putNumber(getName() + "/Arm Extension I", m_armExtensionMotorPidController.getI());
        SmartDashboard.putNumber(getName() + "/Arm Extension D", m_armExtensionMotorPidController.getD());
        SmartDashboard.putNumber(getName() + "/Arm Extension FF", m_armExtensionMotorPidController.getFF());
        SmartDashboard.putNumber(getName() + "/Arm Extension Izone", m_armExtensionMotorPidController.getIZone());
    }

    public void smartDashboardUpdate() {
        m_armRotatorMotorPidController.setP(SmartDashboard.getNumber(getName() + "/Arm Rotator P", kArmRotatorP));
        m_armRotatorMotorPidController.setI(SmartDashboard.getNumber(getName() + "/Arm Rotator I", kArmRotatorI));
        m_armRotatorMotorPidController.setD(SmartDashboard.getNumber(getName() + "/Arm Rotator D", kArmRotatorD));

        m_armExtensionMotorPidController.setP(SmartDashboard.getNumber(getName() + "/Arm Extension P", kArmExtensionP));
        m_armExtensionMotorPidController.setI(SmartDashboard.getNumber(getName() + "/Arm Extension I", kArmExtensionI));
        m_armExtensionMotorPidController.setD(SmartDashboard.getNumber(getName() + "/Arm Extension D", kArmExtensionD));
        m_armExtensionMotorPidController.setFF(SmartDashboard.getNumber(getName() + "/Arm Extension FF", kArmExtensionFF));
        m_armExtensionMotorPidController.setIZone(SmartDashboard.getNumber(getName() + "/Arm Extension Izone", kArmExtensionIzone));
    }

    public void setArmDegrees(double degrees) {
        // trapezoidal testing
        m_goal = new TrapezoidProfile.State(degrees, kTravelVelocity);
        m_armRotatorTrapezoidProfile = new TrapezoidProfile(kConstraints, m_goal, m_setpoint);

        // simple arm PID control
        m_armRotatorMotorPidController.setSetpoint(degrees);

        m_lastArmRotationTarget = degrees;
    }

    public void setArmExtensionPosition(double distance) {
        // m_armExtensionMotor.set(0); // stop motor so us modifying the going forward doesn't screw things up

        // // m_ropeSensor.setGoingForward(distance > m_lastArmExtensionTarget);
        // // m_armExtensionMotorPidController.setSetpoint(distance);
        m_armExtensionMotorPidController.setReference(distance, CANSparkMax.ControlType.kPosition);

        m_lastArmExtensionTarget = distance;
    }

    public void stopArmExtensionMotor() {
        setArmExtensionPosition(getArmExtensionPosition());
    }

    public void stopArmRotatorMotor() {
        setArmDegrees(getArmAngle());
    }

    public void initalizeArm(GamePieceType gamePieceType) {
        if(gamePieceType == GamePieceType.Cone) {
            m_extensionEncoder.setPosition(kLoadedConeExtensionPositionOffset);
        }
        else if(gamePieceType == GamePieceType.Cube) {
            m_extensionEncoder.setPosition(kLoadedCubeExtensionPositionOffset);
        }
    }

    public boolean isArmExtensionAtSetpoint() {
        return Math.abs(m_lastArmExtensionTarget - getArmExtensionPosition()) < 5;
    }

    public boolean isArmRotatorAtSetpoint() {
        return Math.abs(m_lastArmRotationTarget - getArmAngle()) < 5;
    }

    public double getArmAngle() {
        return m_armRotatorEncoderDutyCycle.getOutput() * 360;
    }

    public double getArmExtensionPosition() {
        // return m_ropeSensor.getTicks();
        return m_extensionEncoder.getPosition();
    }

    public void conePickUp() {
        m_gripVacuumMotor.set(1);
        m_armVacuumRelease1.set(false);
        m_armVacuumRelease2.set(false);
        m_armVacuumRelease3.set(false);
        m_armVacuumRelease4.set(true);
        setGamePieceType(GamePieceType.Cone);
    }

    public void cubePickUp() {
        m_gripVacuumMotor.set(1);
        m_armVacuumRelease1.set(true); // TODO check this
        m_armVacuumRelease2.set(true);
        m_armVacuumRelease3.set(true);
        m_armVacuumRelease4.set(false);
        
        // m_armVacuumRelease1.set(false); // TODO check this
        // m_armVacuumRelease2.set(false);
        // m_armVacuumRelease3.set(false);
        // m_armVacuumRelease4.set(true);
        setGamePieceType(GamePieceType.Cube);
    }

    public void gamePieceRelease()
    {
        m_gripVacuumMotor.set(0);
        m_armVacuumRelease1.set(true);
        m_armVacuumRelease2.set(true);
        m_armVacuumRelease3.set(true);
        m_armVacuumRelease4.set(true);
        setGamePieceType(GamePieceType.Unknown);
    }

    public void moveToTargetRotation(double target) {
        m_armRotatorMotor.set(m_armRotatorMotorPidController.calculate(target));
    }

    public void manipulateVacuumRelease(boolean release) {
        manipulateVacuumSolenoids(release);
        if (release) {
            manipulateVacuumMotors(false);
        }
    }

    public void engageVacuum() {
        manipulateVacuumSolenoids(false);
        manipulateVacuumMotors(true);
    }

    private void manipulateVacuumSolenoids(boolean newState) {
        m_armVacuumRelease1.set(newState);
        m_armVacuumRelease2.set(newState);
        m_armVacuumRelease3.set(newState);
        m_armVacuumRelease4.set(newState);
    }

    private void manipulateVacuumMotors(boolean evacuate) {
        var speed = evacuate ? 1.0 : 0.0;
        m_gripVacuumMotor.set(speed);
    }

    public void setGamePieceType(GamePieceType gamePiece) {
        m_GamePieceType = gamePiece;
    }

    public GamePieceType getGamePieceType() {
        return m_GamePieceType;
    } 
}
