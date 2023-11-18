package frc.robot.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.AutoBase;
import frc.robot.subsystems.swerve.SwerveBase;

public class Taxi extends AutoBase {
    public Taxi(SwerveBase swerve) {
        super(swerve);
        PathPlannerTrajectory taxi = PathPlanner.loadPath("taxi", 4.0, 3.0);
        PPSwerveControllerCommand command = baseSwerveCommand(taxi);
        PathPlannerState initialState = taxi.getInitialState();

        addCommands(
                new InstantCommand(() -> SwerveBase.gyro.reset()),
                new InstantCommand(
                        () -> swerve.resetOdometry(new Pose2d(initialState.poseMeters.getTranslation(),
                                initialState.holonomicRotation))),
                command);

    }
}