package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ZeroGyro extends Command{
    private SwerveSubsystem swerveSubsystem;
    
    public ZeroGyro(SwerveSubsystem swerveSubsystem) { 
        this.swerveSubsystem = swerveSubsystem; 
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        swerveSubsystem.zeroHeading();
        SmartDashboard.putBoolean("Zeroed Gyro", true); 
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
    
}
