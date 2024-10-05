package frc.robot.commands.Swerve;

import java.util.function.Supplier;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.IOConstants;

/**
 * This command is responsible for providing proper output to the swerve drive motors based on the controller input.
 */
public class SwerveJoystickCmd extends Command {

    private final SwerveSubsystem swerveSubsystem; // Define the subsystem
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction; // Supplier Doubles to constantly give us the x, y and rotation speed of the robot from the controllers.
    private final Supplier<Boolean> fieldOrientedFunction, isSlowMode; // Supplier booleans to constantly give us whether the robot is field oriented or in slow mode. 
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter; // To limit the rate at which the speed values for the motors increase
    
    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
        Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
        Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> isSlowMode) {

        // Initialzing all the variables we defined. 
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.isSlowMode = isSlowMode; 

        // Initialzing the rate limiters
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        // Get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        // Apply deadband so you don't get input from controllers without doing anything. 
        xSpeed = Math.abs(xSpeed) > IOConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > IOConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > IOConstants.kDeadband ? turningSpeed : 0.0;

        // Make the driving smoother by limiting the rate at which their speed increase. 
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
                
        ChassisSpeeds chassisSpeeds; // Define the chassis speed. 

        if(!isSlowMode.get()) { 
            // Slow Mode not Activated
            chassisSpeeds = fieldOrientedFunction.get() ? ChassisSpeeds.fromFieldRelativeSpeeds( // If slow mode isn't activated run the robot as normal
                xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d()) : new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        } else { 
            // Slow Mode Activated
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds( // Slows down the x and y speed by 50% helps to align the robot from far away. 
                (xSpeed * ModuleConstants.slowModeMultiplier), (ySpeed * ModuleConstants.slowModeMultiplier), turningSpeed, swerveSubsystem.getRotation2d());
        }

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);  // Converts the chassis speed to output for each of the swerve modules. 
        swerveSubsystem.setModuleStates(moduleStates); // outputs the target module states to the swerve modules. 
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