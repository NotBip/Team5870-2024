// package frc.robot.commands.Swerve;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.subsystems.PhotonLL;

// public class AutoAlign extends Command {

//     private SwerveSubsystem swerveSubsystem;
//     private final PhotonLL limelight;
//     private final SlewRateLimiter xLimiter, yLimiter, gyroLimiter;
//     private final PIDController drivePID, strafePID, rotationPID;
//     private final double driveOffset, strafeOffset, rotationOffset;

//     public AutoAlign(SwerveSubsystem swerveSubsystem, PhotonLL limelight) { 
//         this.swerveSubsystem = swerveSubsystem; 
//         this.limelight = limelight; 

//         this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
//         this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
//         this.gyroLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

//         this.drivePID = new PIDController(
//             0.3,
//             0,
//             0);

//         this.strafePID = new PIDController(
//             0.3,
//             0,
//             0);

//         this.rotationPID = new PIDController(
//             0.3,
//             0,
//             0);

//         this.driveOffset = 2.1;
//         this.strafeOffset = -0.2;
//         this.rotationOffset = 10.2;

        
//         addRequirements(swerveSubsystem);
//         addRequirements(limelight);

//     }

//     // Called when the command is initially scheduled.
//     @Override
//     public void initialize() {
//         CameraServer.startAutomaticCapture();
//     }


//     // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute() {
//         double velForward = 0;
//         double velStrafe = 0;
//         double velRot = 0;

//         if(limelight.hasValueTargets()){
//             velForward = drivePID.calculate(limelight.getArea(), driveOffset);
//             velStrafe = strafePID.calculate(limelight.getXDistance(), strafeOffset);
//             velRot = -rotationPID.calculate(limelight.getYaw(), rotationOffset); 
//         } else if(limelight.hasValueTargets() == false){
//             velForward = 0;
//             velStrafe = 0;
//             velRot = 0.4;  
//         } else {
//             velForward = 0;
//             velStrafe = 0;
//             velRot = 0; 
//         }
 
//         // 3. Make the driving smoother
//         velForward = xLimiter.calculate(velForward) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
//         velStrafe = yLimiter.calculate(velStrafe) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
//         velRot = gyroLimiter.calculate(velRot) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
 
//         // 4. Construct desired chassis speeds
//         ChassisSpeeds chassisSpeeds;
         
//         //Relative to robot
//         chassisSpeeds = new ChassisSpeeds(velForward, velStrafe, velRot);

 
//         // 5. Convert chassis speeds to individual module states
//         SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
 
//         // 6. Output each module states to wheels
//         swerveSubsystem.setModuleStates(moduleStates);
//     }


//     // Called once the command ends or is interrupted.
//     @Override
//     public void end(boolean interrupted) {
//         swerveSubsystem.stopModules();
//     }

    
//     // Returns true when the command should end.
//     @Override
//     public boolean isFinished() {
//         return true;
//     }
    
// }
