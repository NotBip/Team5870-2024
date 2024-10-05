package frc.robot.commands.Swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This command is responsible for slightly nudging the robot to the right to help align it. 
 */
public class NudgeRight extends Command {

    private SwerveSubsystem swerveSubsystem; // define the subsystem
    
    public NudgeRight(SwerveSubsystem swerveSubsystem) { 
        this.swerveSubsystem = swerveSubsystem; // Initialize the subsystem. 
        addRequirements(swerveSubsystem); // This command will only run if there aren't anything other commands already running for swerve subsystem. 
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Slightly moving the robot to the right using chassis speeds and then converting them to output for the modules. 
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, .6, 0, swerveSubsystem.getRotation2d());
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
