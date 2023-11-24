package frc.robot.Odometry;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleStates {
    static Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    static Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    static Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    static Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    static SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
  m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
);

    static ChassisSpeeds speeds = new ChassisSpeeds(1.0, 3.0, 1.5);
 static SwerveModuleState[] state = m_kinematics.toSwerveModuleStates(speeds);
    // Front left module state
    public static SwerveModuleState frontLeft = state[0];

    // Front right module state
    SwerveModuleState frontRight = state[1];

    // Back left module state
    SwerveModuleState backLeft = state[2];

    // Back right module state
    SwerveModuleState backRight = state[3];
}
