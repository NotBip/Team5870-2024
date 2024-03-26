package frc.robot.commands.Swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SourceAlign extends Command{
    
    private SwerveSubsystem swerveSubsystem;
    private final SlewRateLimiter gyroLimiter;
    private final PIDController rotationPID;
    private final double rotationOffset;  
    
    public SourceAlign(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem; 
        this.gyroLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

        this.rotationPID = new PIDController(
            0.3,
            0,
            0);

            var alliance = DriverStation.getAlliance(); 
            if (alliance.get() == DriverStation.Alliance.Red) { 
                rotationOffset = 60;
            } else {
                rotationOffset = -60;
            }

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double velRot = -rotationPID.calculate(swerveSubsystem.getHeading(), rotationOffset);
        velRot = gyroLimiter.calculate(velRot) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 4. Construct desired chassis speeds relative to robot
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, velRot);

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
 
        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
