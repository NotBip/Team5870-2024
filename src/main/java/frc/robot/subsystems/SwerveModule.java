package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.ModuleConstants;
//import swervelib.encoders.CANCoderSwerve;
// import frc.robot.commands.SwerveJoystickCmd;

public class SwerveModule {
    // Initalize the Motors. 
    public int modNum; 
    private final CANSparkMax driveMotor; 
    private final CANSparkMax turningMotor; 

    // Initialize the Encoders. 
    private final RelativeEncoder driveEncoder; 
    private final RelativeEncoder turningEncoder;

    // Initialziing PID Controller for turning. 
    PIDController turningPidController;

    // Initalizing ports for encoder. 
    private final CANCoder absoluteEncoder;
    CANcoderConfiguration config = new CANcoderConfiguration();
    private final boolean absoluteEncoderReversed;
    private double absoluteEncoderOffsetRad;
    public double offset = 0; 


    /**
     * Constructor for each Swerve Module. 
     * @param driveMotorId  Port for drive Motor 
     * @param turningMotorId    Port for Turning Motor.
     * @param driveMotorReversed    Boolean if Drive Motor is Reversed. 
     * @param turningMotorReversed  Boolean if Turning Motor is Reversed. 
     * @param absoluteEncoderId     Analog Input Port of Absolute Encoder
     * @param absoluteEncoderOffset absolute Encoder offset
     * @param absoluteEncoderReversed   Boolean if Absolute Encoder is Reversed. 
     */
    public SwerveModule(int modNum, int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed){

        // Set Absolute Encoder Port. 
        this.modNum = modNum; 
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset; 
        this.absoluteEncoderReversed = absoluteEncoderReversed; 
        absoluteEncoder = new CANCoder(absoluteEncoderId);
        // Set drive Motor and turning Motor type and port.
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);
        // Set Motors inverted if true. 
        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        // Get encoder values for both drive and turning motors. 
        driveEncoder = driveMotor.getEncoder(); 
        turningEncoder = turningMotor.getEncoder();
        

        // Convert Encoder values. 
        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
        // Initialzing PID Controller. 
        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        // Absolute Encoder Configs
        // config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        // config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        // absoluteEncoder.getConfigurator().apply(config); 

        // Reset Encoders at the start. 
        resetEncoders();
    }

    /**
     * Get Current Drive motor Position. 
     * @return (Double) Position for the motor. 
     *
    public double getDrivePosition() {
        return driveEncoder.getPosition(); 
    }

    /**
     * Get the Current Turning motor Position. 
     * @return (Double) Position for the turning Motor. 
     */
    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    /**
     * Get the Speed for the Drive Motor. 
     * @return (Double) Speed of the Motor. 
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * Get the Turning Motor Speed. 
     * @return (Double) Speed of the motor. 
     */
    public double getTurningVelocity() {
        return turningEncoder.getVelocity(); 
    }

    /**
     * get AbsoluteEncoder's Value in radians. 
     * @return (Double) Absolute Encoder vol in rads. 
     */
    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getAbsolutePosition();
        angle *= (Math.PI/180);
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }


    /**
     * Reset the Encoders. 
     */
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }
    
    /**
     * Gets the current state (speed and Position) of a wheel. 
     * @return State of the wheel. 
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    /**
     * Sets the desired State of the Robots (Angles and Speed). 
     * @param state
     */
    public void setDesiredState(SwerveModuleState state, String wheel) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    public SwerveModulePosition getPositions(){ 
        return new SwerveModulePosition(
            driveEncoder.getPosition(), 
            getState().angle);
    }


    /** 
     * Stops the Drive Motor and Turning Motor. 
     */
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

}
