package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.util.swerveUtil.CTREModuleState;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;

import frc.robot.RobotMap;

// import static frc.robot.RobotMap.Swerve.swerveCANcoderConfig;

public class RevSwerveModule implements SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    // private Rotation2d lastAngle;

    private CANSparkMax mAngleMotor;
    private CANSparkMax mDriveMotor;

    private RelativeEncoder angleEncoder; // CHANGE TO CANCoder - NEW PARTS? SWITCH TO CAN BUS?
    private RelativeEncoder relAngleEncoder;
    private RelativeEncoder relDriveEncoder;

    public SwerveModuleState desiredState;

    public RevSwerveModule(int moduleNumber, RevSwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID,  MotorType.kBrushless);
        configDriveMotor();

        /* Angle Encoder Config */
        angleEncoder = mDriveMotor.getEncoder();
        configEncoders();
    }



    private void configEncoders() {
        // absolute encoder
        // angleEncoder.configAllSettings(swerveCANcoderConfig);

        relDriveEncoder = mDriveMotor.getEncoder();
        relDriveEncoder.setPosition(0);


        relDriveEncoder.setPositionConversionFactor(RobotMap.Swerve.driveRevToMeters);
        relDriveEncoder.setVelocityConversionFactor(RobotMap.Swerve.driveRpmToMetersPerSecond);


        relAngleEncoder = mAngleMotor.getEncoder();
        relAngleEncoder.setPositionConversionFactor(RobotMap.Swerve.DegreesPerTurnRotation);
        // in degrees/sec
        relAngleEncoder.setVelocityConversionFactor(RobotMap.Swerve.DegreesPerTurnRotation / 60);

        synchronizeEncoders();
        mDriveMotor.burnFlash();
        mAngleMotor.burnFlash();
    }

    private void configAngleMotor() {
        mAngleMotor.restoreFactoryDefaults();
        SparkMaxPIDController controller = mAngleMotor.getPIDController();
        controller.setP(RobotMap.Swerve.angleKP, 0);
        controller.setI(RobotMap.Swerve.angleKI, 0);
        controller.setD(RobotMap.Swerve.angleKD, 0);
        controller.setFF(RobotMap.Swerve.angleKFF, 0);
        controller.setOutputRange(-RobotMap.Swerve.anglePower, RobotMap.Swerve.anglePower);
        mAngleMotor.setSmartCurrentLimit(RobotMap.Swerve.angleContinuousCurrentLimit);

        mAngleMotor.setInverted(RobotMap.Swerve.angleMotorInvert);
        mAngleMotor.setIdleMode(RobotMap.Swerve.angleIdleMode);
        mAngleMotor.setClosedLoopRampRate(RobotMap.Swerve.angleRampRate);
    }

    private void configDriveMotor() {
        mDriveMotor.restoreFactoryDefaults();
        SparkMaxPIDController controller = mDriveMotor.getPIDController();
        controller.setP(RobotMap.Swerve.driveKP, 0);
        controller.setI(RobotMap.Swerve.driveKI, 0);
        controller.setD(RobotMap.Swerve.driveKD, 0);
        controller.setFF(RobotMap.Swerve.driveKFF, 0);
        controller.setOutputRange(-RobotMap.Swerve.drivePower, RobotMap.Swerve.drivePower);
        mDriveMotor.setSmartCurrentLimit(RobotMap.Swerve.driveContinuousCurrentLimit);
        mDriveMotor.setInverted(RobotMap.Swerve.driveMotorInvert);
        mDriveMotor.setIdleMode(RobotMap.Swerve.driveIdleMode);
    }



    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        // CTREModuleState actually works for any type of motor.
        this.desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        setAngle(this.desiredState);
        setSpeed(this.desiredState, isOpenLoop);

        if(mDriveMotor.getFault(FaultID.kSensorFault)) {
            DriverStation.reportWarning("Sensor Fault on Drive Motor ID:" + mDriveMotor.getDeviceId(), false);
        }

        if(mAngleMotor.getFault(FaultID.kSensorFault)) {
            DriverStation.reportWarning("Sensor Fault on Angle Motor ID:" + mAngleMotor.getDeviceId(), false);
        }
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if(isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / RobotMap.Swerve.maxSpeed;
            mDriveMotor.set(percentOutput);
            return;
        }

        double velocity = desiredState.speedMetersPerSecond;

        SparkMaxPIDController controller = mDriveMotor.getPIDController();
        controller.setReference(velocity, ControlType.kVelocity, 0);
    }

    private void setAngle(SwerveModuleState desiredState) {
        if(Math.abs(desiredState.speedMetersPerSecond) <= (RobotMap.Swerve.maxSpeed * 0.01)) {
            mAngleMotor.stopMotor();
            return;

        }
        Rotation2d angle = desiredState.angle;
        double degReference = angle.getDegrees();
        // Prevent rotating module if speed is less then 1%. Prevents Jittering.

        SparkMaxPIDController controller = mAngleMotor.getPIDController();
        controller.setReference(degReference, ControlType.kPosition, 0);
    }



    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(relAngleEncoder.getPosition());
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getPosition());
        //return getAngle();
    }

    public int getModuleNumber() {
        return moduleNumber;
    }

    public void setModuleNumber(int moduleNumber) {
        this.moduleNumber = moduleNumber;
    }

    public void synchronizeEncoders() {
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        relAngleEncoder.setPosition(absolutePosition);
    }



    public SwerveModuleState getState() {
        return new SwerveModuleState(
                relDriveEncoder.getVelocity(),
                getAngle()
        );
    }

    public double getOmega() {
        return angleEncoder.getVelocity() / 360;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                relDriveEncoder.getPosition(),
                getAngle()
        );
    }
}
