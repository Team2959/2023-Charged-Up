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

public class SwerveModule {
    public static final double kDriveKp = 0.05;
    public static final double kDriveKi = 0.0;
    public static final double kDriveKd = 0.001;
    public static final double kDriveFf = 0.02;
    public static final double kDriveIzone = 600;
    public static final double kTurnKp = 0.4;
    public static final double kTurnKi = 0.00001;
    public static final double kTurnKd = 0.0;
    public static final double kTurnFf = 0.0;
    public static final double kTurnIzone = 1.0;

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
    private static final double kGearboxRatio = 1.0 / 6.86; // One turn of the wheel is 6.86 turns of the motor
    private static final double kDrivePositionFactor = (2.0 * Math.PI * kWheelRadius * kGearboxRatio);
    private static final double kDriveCurrentLimitAmps = 80.0;
    private static final double kTurnCurrentLimitAmps = 20.0; 

    public SwerveModule(int driveMotor, int turnMotor, int dutyCycle, double turnOffset, String name)
    {
        m_driveMotor = new CANSparkMax(driveMotor, CANSparkMax.MotorType.kBrushless);
        m_turnMotor = new CANSparkMax(turnMotor, CANSparkMax.MotorType.kBrushless);
        m_driveMotor.restoreFactoryDefaults();
        m_turnMotor.restoreFactoryDefaults();

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
        
        m_drivePIDController.setP(kDriveKp);
        m_drivePIDController.setI(kDriveKi);
        m_drivePIDController.setD(kDriveKd);
        m_drivePIDController.setFF(kDriveFf);
        m_drivePIDController.setIZone(kDriveIzone);

        m_turnPIDController.setFeedbackDevice(m_turnEncoder);
        m_turnPIDController.setP(kTurnKp);
        m_turnPIDController.setI(kTurnKi);
        m_turnPIDController.setD(kTurnKd);
        m_turnPIDController.setIZone(kTurnIzone);

        m_turnEncoder.setPositionConversionFactor(2.0 * Math.PI);
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

        if(Math.abs(state.speedMetersPerSecond - 0) < 0.001)
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
}


