package frc.robot.subsystems.swerve;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.RobotMap;

/**
 * Parent class for all autonomous commands
 */
public class AutoBase extends SequentialCommandGroup {
    public SwerveBase swerve;
    public static final ProfiledPIDController profiledthetaController =
        new ProfiledPIDController(RobotMap.AutoConstants.THETA_kP, 0, 0,
            RobotMap.AutoConstants.kThetaControllerConstraints);
    public static final PIDController thetaController =
        new PIDController(RobotMap.AutoConstants.THETA_kP, 0, 0);
    public static final PIDController pidX =
        new PIDController(RobotMap.AutoConstants.X_kP, 0, 0);
    public static final PIDController pidY =
        new PIDController(RobotMap.AutoConstants.Y_kP, 0, 0);

    /**
     * Autonomous that aligns limelight then executes a trajectory.
     *
     * @param swerve swerve subsystem
     */
    public AutoBase(SwerveBase swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Creates a SwerveControllerCommand from a Trajectory
     *
     * @param trajectory Trajectory to run
     * @return A SwerveControllerCommand for the robot to move
     */
    public SwerveControllerCommand baseSwerveCommand(Trajectory trajectory) {
        SwerveControllerCommand command = new SwerveControllerCommand(trajectory, swerve::getPose,
        RobotMap.Swerve.swerveKinematics,
                new PIDController(RobotMap.AutoConstants.X_kP, 0, 0),
                new PIDController(RobotMap.AutoConstants.Y_kP, 0, 0),
                new ProfiledPIDController(RobotMap.AutoConstants.THETA_kP, 0, 0,
                RobotMap.AutoConstants.kThetaControllerConstraints),
                swerve::setModuleStates, swerve);
        return command;
    }

    /**
     * Creates a SwerveController Command using a Path Planner Trajectory
     *
     * @param trajectory a Path Planner Trajectory
     * @return A SwerveControllerCommand for the robot to move
     */
    public PPSwerveControllerCommand baseSwerveCommand(PathPlannerTrajectory trajectory) {
        PPSwerveControllerCommand command = new PPSwerveControllerCommand(trajectory, swerve::getPose, RobotMap.Swerve.swerveKinematics, pidX, pidY, thetaController, swerve::setModuleStates, swerve);
        return command;
    }
}