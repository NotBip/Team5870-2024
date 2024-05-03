package frc.robot.commands.Swerve;

import java.util.function.Supplier;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;

public class SwerveJoystickCmd extends Command{

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction, isSlowMode, sourceAlign;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private final PIDController rotationPID;
    
    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
        Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
        Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> isSlowMode, Supplier<Boolean> sourceAlign) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.isSlowMode = isSlowMode; 
        this.sourceAlign = sourceAlign;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

        this.rotationPID = new PIDController(
            0.05,
            0,
            0);



        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();
        double rotationOffset; 

        var alliance = DriverStation.getAlliance(); 
        if (alliance.get() == DriverStation.Alliance.Red) { 
            rotationOffset = 60;
        } else {
            rotationOffset = -60;
        }

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
                
        // For Source Aligning
        double velRot = -rotationPID.calculate(swerveSubsystem.getRotation2d().getDegrees() , rotationOffset);

        ChassisSpeeds chassisSpeeds;

        if(!isSlowMode.get()) { 
            
            // If not alligning for source drive normally
            if(!sourceAlign.get()) {
                chassisSpeeds = fieldOrientedFunction.get() ? ChassisSpeeds.fromFieldRelativeSpeeds(
                            xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d()) : new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
            } else { 
                // align to source while driving
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, velRot, swerveSubsystem.getRotation2d()); 
            }

        } else { 
            // Activate Slow Mode
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        (xSpeed * ModuleConstants.slowModeMultiplier), (ySpeed * ModuleConstants.slowModeMultiplier), turningSpeed, swerveSubsystem.getRotation2d());
        }

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}