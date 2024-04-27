package frc.robot.commands;

import java.util.function.Supplier;
import frc.robot.subsystems.SwerveSim;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

public class SwerveJoystickCmd extends Command{

    private final SwerveSim swerveSim;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction, zSpdFuntion, zSpdFuntion2;
    private final Supplier<Boolean> fieldOrientedFunction, reversed;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    
    public SwerveJoystickCmd(SwerveSim swerveSim, Supplier<Double> zSpdFunction, Supplier<Double> zSpdFunction2, Supplier<Boolean> reversed,
        Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
        Supplier<Boolean> fieldOrientedFunction) {
        this.swerveSim = swerveSim;
        this.zSpdFuntion2 = zSpdFunction2; 
        this.zSpdFuntion = zSpdFunction; 
        this.reversed = reversed; 
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSim);
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

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
                
        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            // Relative to field
            swerveSim.driveFieldRelative(new ChassisSpeeds(xSpeed, ySpeed, turningSpeed));
       } else {
            // Relative to robot
            swerveSim.driveFieldRelative(new ChassisSpeeds(xSpeed*.5, ySpeed*.5, turningSpeed)); 
        }

        if(zSpdFuntion.get() >= 1 || zSpdFuntion2.get() >= 1){ 
            swerveSim.updatePose3d(reversed.get(), .01); 

        }

        }


    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}