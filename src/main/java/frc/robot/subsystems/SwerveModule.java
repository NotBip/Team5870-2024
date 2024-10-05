package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

/**
 * This class is used to initalize each swerve modules. (REPLACE THE DEPRECATED CLASS IMPORT) 
 */
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
    private final CANCoder absoluteEncoder; // Need to replace this with the new class for 2025. 
    private final boolean absoluteEncoderReversed;
    private double absoluteEncoderOffsetRad;

    /**g
     * Constructor for each Swerve Module. 
     * @param driveMotorId  Port for drive Motor 
     * @param turningMotorId    Port for Turning Motor.
     * @param driveMotorReversed    Boolean if Drive Motor is Reversed. 
     * @param turningMotorReversed  Boolean if Turning Motor is Reversed. 
     * @param absoluteEncoderId     Analog Input Port of Absolute Encoder
     * @param absoluteEncoderOffset absolute Encoder offset
     * @param absoluteEncoderReversed   Boolean if Absolute Encoder is Reversed. 
     */
    public SwerveModule(int modNum, int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, 
    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed){

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
        

        // Convert Encoder values from rotations to actual measurements we can use. 
        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        // Initialzing PID Controller. 
        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        // Reset Encoders at the start. 
        resetEncoders();
    }

   /**
    * This method uses the turning encoder to get the position of the turning motor. 
    * @return The position of the turning motor in radians.
    */
    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    /**
     * This method uses the driveEncoder to get the velocity of the drive motor. 
     * @return The velocity of the drive motor in Meters per seconds.
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * This method uses the turning encoder to get the velocity at which the turning motor moves. 
     * @return The speed at which the turning motor moves in Radians per sec. 
     */
    public double getTurningVelocity() {
        return turningEncoder.getVelocity(); 
    }

    /**
     * Uses the absolute encoder for the module to figure of the angle at which the wheel is facing relative to the front defined in the constants class using the 
     * absolute encoder offsets. 
     * @return The angle of the wheel in radians. 
     */
    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getAbsolutePosition();
        angle *= (Math.PI/180); // Convert to radians
        angle -= absoluteEncoderOffsetRad; // Subtract the offset so they are all facing the same direction at angle 0;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

   /**
    * This method is used to reset both the drive and turning encoder. The drive encoder is set to 0 and the turning encoder is set to the current angle of the wheel
    * using getAbsoluteEncoderRad()
    */
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }
    
    /**
     * This method is used to get the state of the swerve module. 
     * @return The current state of the swerve module which contains the velocity and rotation2D. 
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    /**
     * This method takes in the new state of the swerve module and figures out to most optimal way to achieve that state using the optimize function from WPILib. 
     * it then outputs the speed to the motors which is further optimized with a PID Controller.
     * @param state The new state of the swerve module. 
     * @param wheel The position of the wheel (It is used for debugging mostly not needed)
     */
    public void setDesiredState(SwerveModuleState state, String wheel) { 
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle); // figures out the fastest way for the motor to go to it's new position
        driveMotor.set(state.speedMetersPerSecond / Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond); // setting the speed for the drive motor. 
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians())); // calculating the speed for the turning motor to achieve the required angle and outputs it. 
    }

    /**
     * This method is used to get the positon of the swerve module. 
     * @return The positon of the swerve module which contains the drive Encoder's position in meters and the angle at which the wheel is facing 
     */
    public SwerveModulePosition getPositions(){ 
        return new SwerveModulePosition(
            driveEncoder.getPosition(), 
            getState().angle);
    }

    /**
     * This method is used to stop both the turning and drive motor by setting their speed to 0 for the swerve module. 
     */
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

}
