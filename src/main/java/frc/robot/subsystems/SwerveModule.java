package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

public class SwerveModule {
    private static final double kDriveP = 0.05;
    private static final double kDriveI = 0.0;
    private static final double kDriveD = 0.001;
    private static final double kDriveFF = 0.02;
    private static final double kDriveIZone = 600;
    private static final double kTurnP = 0.4;
    private static final double kTurnI = 0.00001;
    private static final double kTurnD = 0.0;
    private static final double kTurnFF = 0.0;
    private static final double kTurnIZone = 1.0;

    private CANSparkMax m_driveMotor;
    private CANSparkMax m_turnMotor;
    private DigitalInput m_dutyCycleInput;
    private DutyCycle m_dutyCycleEncoder;
    private SparkMaxRelativeEncoder m_driveEncoder;
    private SparkMaxAlternateEncoder m_turnEncoder;
    private SparkMaxPIDController m_drivePIDController;
    private SparkMaxPIDController m_turnPIDController;
    private final double m_turnOffset;
    private final String m_name;

    private static final double kWheelRadius = 2.0 * 0.0254; // 2" * 0.0254 m / inch
    private static final int kEncoderResolution = 4096;
    private static final double kGearboxRatio = 1.0 / 6.12; // One turn of the wheel is 6.86 turns of the motor
    private static final double kDrivePositionFactor = (2.0 * Math.PI * kWheelRadius * kGearboxRatio);
    private static final double kDriveCurrentLimitAmps = 80.0;
    private static final double kTurnCurrentLimitAmps = 20.0; 

    public SwerveModule(int driveMotor, int turnMotor, int dutyCycle, double turnOffset, String name)
    {
        m_driveMotor = new CANSparkMax(driveMotor, CANSparkMax.MotorType.kBrushless);
        m_turnMotor = new CANSparkMax(turnMotor, CANSparkMax.MotorType.kBrushless);
        m_driveMotor.restoreFactoryDefaults();
        m_turnMotor.restoreFactoryDefaults();

        m_driveMotor.setIdleMode(IdleMode.kBrake);
        m_turnMotor.setIdleMode(IdleMode.kBrake);

        m_driveMotor.setSmartCurrentLimit((int) kDriveCurrentLimitAmps);
        m_turnMotor.setSmartCurrentLimit((int) kTurnCurrentLimitAmps);

        m_name = name;

        m_turnOffset = turnOffset;

        m_dutyCycleInput = new DigitalInput(dutyCycle);
        m_dutyCycleEncoder = new DutyCycle(m_dutyCycleInput);

        m_driveEncoder = (SparkMaxRelativeEncoder) m_driveMotor.getEncoder();
        m_turnEncoder = (SparkMaxAlternateEncoder) m_turnMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, kEncoderResolution);
    
        m_drivePIDController = m_driveMotor.getPIDController();
        m_turnPIDController = m_turnMotor.getPIDController();

        m_driveEncoder.setPositionConversionFactor(kDrivePositionFactor);
        m_driveEncoder.setVelocityConversionFactor(kDrivePositionFactor / 60.0);
        
        m_drivePIDController.setP(kDriveP);
        m_drivePIDController.setI(kDriveI);
        m_drivePIDController.setD(kDriveD);
        m_drivePIDController.setFF(kDriveFF);
        m_drivePIDController.setIZone(kDriveIZone);

        m_turnPIDController.setFeedbackDevice(m_turnEncoder);
        m_turnPIDController.setP(kTurnP);
        m_turnPIDController.setI(kTurnI);
        m_turnPIDController.setD(kTurnD);
        m_turnPIDController.setIZone(kTurnIZone);

        m_turnEncoder.setPositionConversionFactor(2.0 * Math.PI);
    }

    public void driveSmartDashboardInit() {
        SmartDashboard.putNumber(m_name + "/Drive P", m_drivePIDController.getP());
        SmartDashboard.putNumber(m_name + "/Drive I", m_drivePIDController.getI());
        SmartDashboard.putNumber(m_name + "/Drive D", m_drivePIDController.getD());
        SmartDashboard.putNumber(m_name + "/Drive IZone", m_drivePIDController.getIZone());
        SmartDashboard.putNumber(m_name + "/Drive FF", m_drivePIDController.getFF());
        SmartDashboard.putNumber(m_name + "/Turn P", m_turnPIDController.getP());
        SmartDashboard.putNumber(m_name + "/Turn I", m_turnPIDController.getI());
        SmartDashboard.putNumber(m_name + "/Turn D", m_turnPIDController.getD());
        SmartDashboard.putNumber(m_name + "/Turn IZone", m_turnPIDController.getIZone());
        SmartDashboard.putNumber(m_name + "/Turn FF", m_turnPIDController.getFF());
    }

    public void smartDashboardUpdate() {
        SmartDashboard.putNumber(m_name + "/Drive Encoder Velocity", m_driveEncoder.getVelocity());
        SmartDashboard.putNumber(m_name + "/Drive Encoder Position", m_driveEncoder.getPosition()); 
        SmartDashboard.putNumber(m_name + "/Turn Motor Position", m_turnEncoder.getPosition());
        SmartDashboard.putNumber(m_name + "/Turn Digital IO", m_dutyCycleEncoder.getOutput());

       m_drivePIDController.setP (SmartDashboard.getNumber(m_name + "/Drive P", kDriveP));
       m_drivePIDController.setI (SmartDashboard.getNumber(m_name + "/Drive I", kDriveI));
       m_drivePIDController.setD (SmartDashboard.getNumber(m_name + "/Drive D", kDriveD));
       m_drivePIDController.setIZone (SmartDashboard.getNumber(m_name + "/Drive IZone", kDriveIZone));
       m_drivePIDController.setFF (SmartDashboard.getNumber(m_name + "/Drive FF", kDriveFF));

       m_turnPIDController.setP (SmartDashboard.getNumber(m_name + "/Turn P", kTurnP));
       m_turnPIDController.setI (SmartDashboard.getNumber(m_name + "/Turn I", kTurnI));
       m_turnPIDController.setD (SmartDashboard.getNumber(m_name + "/Turn D", kTurnD));
       m_turnPIDController.setIZone (SmartDashboard.getNumber(m_name + "/Turn IZone", kTurnIZone));
       m_turnPIDController.setFF (SmartDashboard.getNumber(m_name + "/Turn FF", kTurnFF));
    }

    public double getAbsoluteEncoderPosition()
    {
        double initalPosition = m_dutyCycleEncoder.getOutput();
        double initalPositionInRadians = initalPosition * 2.0 * Math.PI;
        double initalPositionInRadiansScaled = new Rotation2d(initalPositionInRadians - m_turnOffset).getRadians();
        return initalPositionInRadiansScaled;
    }

    public double getVelocity()
    {
        return m_driveEncoder.getVelocity();
    }

    public SwerveModulePosition getPosition()
    {
        return new SwerveModulePosition(m_driveEncoder.getPosition(), new Rotation2d(m_turnEncoder.getPosition()));
    }

    /*
     * Sets this modules 
     */
    public void setDesiredState(SwerveModuleState referenceState)
    {
        SwerveModuleState state = SwerveModuleState.optimize(referenceState, new Rotation2d(m_turnEncoder.getPosition()));

        m_drivePIDController.setReference(state.speedMetersPerSecond * DriveSubsystem.kMaxSpeedMetersPerSecond, CANSparkMax.ControlType.kVelocity);

        // if(Math.abs(state.speedMetersPerSecond - 0) < 0.001)
        {
            // Leave because we don't want wheel to go back to zero, because we are stopped
            // return;
        }

        var currentPosition = m_turnEncoder.getPosition();
        Rotation2d delta = state.angle.minus(new Rotation2d(currentPosition));
        double setpoint = currentPosition + delta.getRadians();
        SmartDashboard.putNumber(m_name + "/Setpoint", setpoint);
        SmartDashboard.putNumber(m_name + "/State Angle", state.angle.getRadians());
        m_turnPIDController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
    }

    /*
     * Resets the SparkMax Alternative Encoder to match the absolute Mag encoder,
     * setting the position of the Mag Encoder to the SparkMax Alternative Encoder 
     */
    public void resetAngleEncoderToAbsolute()
    {
        m_turnEncoder.setPosition(getAbsoluteEncoderPosition());
    }

    public void setAngle(double i) {
        m_turnPIDController.setP(i);
    }
}


