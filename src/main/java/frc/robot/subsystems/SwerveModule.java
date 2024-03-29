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
    private static final double kSteerP = 0.4;
    private static final double kSteerI = 0.00001;
    private static final double kSteerD = 0.0;
    private static final double kSteerFF = 0.0;
    private static final double kSteerIZone = 1.0;

    private CANSparkMax m_driveMotor;
    private CANSparkMax m_steerMotor;
    private DigitalInput m_dutyCycleInput;
    private DutyCycle m_dutyCycleEncoder;
    private SparkMaxRelativeEncoder m_driveEncoder;
    private SparkMaxAlternateEncoder m_steerEncoder;
    private SparkMaxPIDController m_drivePIDController;
    private SparkMaxPIDController m_steerPIDController;
    private final double m_steerOffset;
    private final String m_name;

    private static final double kWheelRadius = 2.0 * 0.0254; // 2" * 0.0254 m / inch
    private static final int kEncoderResolution = 4096;
    private static final double kGearboxRatio = 1.0 / 6.12; // One turn of the wheel is 6.86 turns of the motor
    private static final double kDrivePositionFactor = (2.0 * Math.PI * kWheelRadius * kGearboxRatio);
    private static final int kDriveCurrentLimitAmps = 70;
    private static final int kSteerCurrentLimitAmps = 40; 

    public SwerveModule(int driveMotor, int steerMotor, int dutyCycle, double steerOffset, String name)
    {
        m_driveMotor = new CANSparkMax(driveMotor, CANSparkMax.MotorType.kBrushless);
        m_steerMotor = new CANSparkMax(steerMotor, CANSparkMax.MotorType.kBrushless);
        m_driveMotor.restoreFactoryDefaults();
        m_steerMotor.restoreFactoryDefaults();

        m_driveMotor.setIdleMode(IdleMode.kBrake);
        m_steerMotor.setIdleMode(IdleMode.kBrake);

        m_driveMotor.setSmartCurrentLimit(kDriveCurrentLimitAmps);
        m_steerMotor.setSmartCurrentLimit(kSteerCurrentLimitAmps);

        m_name = name;

        m_steerOffset = steerOffset;

        m_dutyCycleInput = new DigitalInput(dutyCycle);
        m_dutyCycleEncoder = new DutyCycle(m_dutyCycleInput);

        m_driveEncoder = (SparkMaxRelativeEncoder) m_driveMotor.getEncoder();
        m_steerEncoder = (SparkMaxAlternateEncoder) m_steerMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, kEncoderResolution);
    
        m_drivePIDController = m_driveMotor.getPIDController();
        m_steerPIDController = m_steerMotor.getPIDController();

        m_driveEncoder.setPositionConversionFactor(kDrivePositionFactor);
        m_driveEncoder.setVelocityConversionFactor(kDrivePositionFactor / 60.0);
        
        m_drivePIDController.setP(kDriveP);
        m_drivePIDController.setI(kDriveI);
        m_drivePIDController.setD(kDriveD);
        m_drivePIDController.setFF(kDriveFF);
        m_drivePIDController.setIZone(kDriveIZone);

        m_steerPIDController.setFeedbackDevice(m_steerEncoder);
        m_steerPIDController.setP(kSteerP);
        m_steerPIDController.setI(kSteerI);
        m_steerPIDController.setD(kSteerD);
        m_steerPIDController.setIZone(kSteerIZone);

        m_steerEncoder.setPositionConversionFactor(2.0 * Math.PI);
    }

    public void smartDashboardInit() {
        SmartDashboard.putNumber(m_name + "/Drive P", m_drivePIDController.getP());
        SmartDashboard.putNumber(m_name + "/Drive I", m_drivePIDController.getI());
        SmartDashboard.putNumber(m_name + "/Drive D", m_drivePIDController.getD());
        SmartDashboard.putNumber(m_name + "/Drive IZone", m_drivePIDController.getIZone());
        SmartDashboard.putNumber(m_name + "/Drive FF", m_drivePIDController.getFF());
        SmartDashboard.putNumber(m_name + "/Turn P", m_steerPIDController.getP());
        SmartDashboard.putNumber(m_name + "/Turn I", m_steerPIDController.getI());
        SmartDashboard.putNumber(m_name + "/Turn D", m_steerPIDController.getD());
        SmartDashboard.putNumber(m_name + "/Turn IZone", m_steerPIDController.getIZone());
        SmartDashboard.putNumber(m_name + "/Turn FF", m_steerPIDController.getFF());
    }

    public void smartDashboardUpdate() {
        SmartDashboard.putNumber(m_name + "/Drive Encoder Velocity", m_driveEncoder.getVelocity());
        SmartDashboard.putNumber(m_name + "/Drive Encoder Position", m_driveEncoder.getPosition()); 
        SmartDashboard.putNumber(m_name + "/Turn Motor Position", m_steerEncoder.getPosition());
        SmartDashboard.putNumber(m_name + "/Turn Digital IO", m_dutyCycleEncoder.getOutput());

       m_drivePIDController.setP (SmartDashboard.getNumber(m_name + "/Drive P", kDriveP));
       m_drivePIDController.setI (SmartDashboard.getNumber(m_name + "/Drive I", kDriveI));
       m_drivePIDController.setD (SmartDashboard.getNumber(m_name + "/Drive D", kDriveD));
       m_drivePIDController.setIZone (SmartDashboard.getNumber(m_name + "/Drive IZone", kDriveIZone));
       m_drivePIDController.setFF (SmartDashboard.getNumber(m_name + "/Drive FF", kDriveFF));

       m_steerPIDController.setP (SmartDashboard.getNumber(m_name + "/Turn P", kSteerP));
       m_steerPIDController.setI (SmartDashboard.getNumber(m_name + "/Turn I", kSteerI));
       m_steerPIDController.setD (SmartDashboard.getNumber(m_name + "/Turn D", kSteerD));
       m_steerPIDController.setIZone (SmartDashboard.getNumber(m_name + "/Turn IZone", kSteerIZone));
       m_steerPIDController.setFF (SmartDashboard.getNumber(m_name + "/Turn FF", kSteerFF));
    }

    public double getAbsoluteEncoderPosition()
    {
        double initalPosition = m_dutyCycleEncoder.getOutput();
        double initalPositionInRadians = initalPosition * 2.0 * Math.PI;
        double initalPositionInRadiansScaled = new Rotation2d(initalPositionInRadians - m_steerOffset).getRadians();
        return initalPositionInRadiansScaled;
    }

    // public void setSmartMotion(double maxVel, double maxAccel) {
    //     m_drivePIDController.setSmartMotionMaxVelocity(maxVel, 0);
    //     m_drivePIDController.setSmartMotionMaxAccel(maxAccel, 0);
    // }

    public double getVelocity()
    {
        return m_driveEncoder.getVelocity();
    }

    public SwerveModulePosition getPosition()
    {
        return new SwerveModulePosition(m_driveEncoder.getPosition(), new Rotation2d(m_steerEncoder.getPosition()));
    }

    /*
     * Sets this modules 
     */
    public void setDesiredState(SwerveModuleState referenceState)
    {
        SwerveModuleState state = SwerveModuleState.optimize(referenceState, new Rotation2d(m_steerEncoder.getPosition()));

        setDriveVelocity(state.speedMetersPerSecond);

        // if(Math.abs(state.speedMetersPerSecond - 0) < 0.001)
        {
            // Leave because we don't want wheel to go back to zero, because we are stopped
            // return;
        }

        var currentAngleInRadians = m_steerEncoder.getPosition();
        Rotation2d delta = state.angle.minus(new Rotation2d(currentAngleInRadians));
        SmartDashboard.putNumber(m_name + "/State Angle", state.angle.getRadians());
        double angleSetpointInRadians = currentAngleInRadians + delta.getRadians();
        setSteerAngleInRadians(angleSetpointInRadians);
    }

    private void setDriveVelocity(double targetSpeed)
    {
        m_drivePIDController.setReference(targetSpeed * DriveSubsystem.kMaxSpeedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        // m_drivePIDController.setReference(targetSpeed * DriveSubsystem.kMaxSpeedMetersPerSecond, CANSparkMax.ControlType.kSmartVelocity);
    }

    private void setSteerAngleInRadians(double targetAngleInRadians)
    {
        SmartDashboard.putNumber(m_name + "/Setpoint", targetAngleInRadians);
        m_steerPIDController.setReference(targetAngleInRadians, CANSparkMax.ControlType.kPosition);
    }

    /*
     * Resets the SparkMax Alternative Encoder to match the absolute Mag encoder,
     * setting the position of the Mag Encoder to the SparkMax Alternative Encoder 
     */
    public void resetAngleEncoderToAbsolute()
    {
        m_steerEncoder.setPosition(getAbsoluteEncoderPosition());
    }

    public void lockWheelAtAngleInDegrees(double degrees)
    {
        setDriveVelocity(0);
        var angleInRadians = degrees * Math.PI / 180.0;
        setSteerAngleInRadians(angleInRadians);
    }
}
