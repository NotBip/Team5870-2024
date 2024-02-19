package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.hal.simulation.EncoderDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.proto.Kinematics;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Autos.blueAmp;



public class SwerveSubsystem extends SubsystemBase {
    
    private Field2d field = new Field2d(); 
    private AHRS navx = new AHRS(SPI.Port.kMXP);
    public SwerveModule[] SwerveMods;
    private SwerveDriveOdometry odometer; 

    public SwerveSubsystem(){
        new Thread(() -> {
            try{
                Thread.sleep(1000);
                 zeroHeading();
            } catch (Exception e){}
        }).start();

            SwerveMods = new SwerveModule[] {
                new SwerveModule(
                    0,
                    DriveConstants.kFrontLeftDriveMotorPort,
                    DriveConstants.kFrontLeftTurningMotorPort,
                    DriveConstants.kFrontLeftDriveEncoderReversed,
                    DriveConstants.kFrontLeftTurningEncoderReversed,
                    DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
                    DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
                    DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed),
                new SwerveModule(
                    1,
                    DriveConstants.kFrontRightDriveMotorPort,
                    DriveConstants.kFrontRightTurningMotorPort,
                    DriveConstants.kFrontRightDriveEncoderReversed,
                    DriveConstants.kFrontRightTurningEncoderReversed,
                    DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
                    DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
                    DriveConstants.kFrontRightDriveAbsoluteEncoderReversed),
                new SwerveModule(
                    2,
                    DriveConstants.kBackLeftDriveMotorPort,
                    DriveConstants.kBackLeftTurningMotorPort,
                    DriveConstants.kBackLeftDriveEncoderReversed,
                    DriveConstants.kBackLeftTurningEncoderReversed,
                    DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
                    DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
                    DriveConstants.kBackLeftDriveAbsoluteEncoderReversed),
                new SwerveModule(
                    3,
                    DriveConstants.kBackRightDriveMotorPort,
                    DriveConstants.kBackRightTurningMotorPort,
                    DriveConstants.kBackRightDriveEncoderReversed,
                    DriveConstants.kBackRightTurningEncoderReversed,
                    DriveConstants.kBackRightDriveAbsoluteEncoderPort,
                    DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
                    DriveConstants.kBackRightDriveAbsoluteEncoderReversed)
        };

        odometer = new SwerveDriveOdometry(Constants.DriveConstants.kDriveKinematics, new Rotation2d(0), getModulePositions());

        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::resetOdometry, 
            this::getSpeeds, 
            this::driveRobotRelative, 
            AutoConstants.pathFollowerConfig, 
            () -> { 
                var alliance = DriverStation.getAlliance(); 
                if(alliance.isPresent()) { 
                    return alliance.get() == DriverStation.Alliance.Red; 
                }
                return false;
            }, this);




    }

    /**
     * Reset Gyro heading usually only during the initializing. 
     */
    public void zeroHeading() {
        navx.reset(); 
    }

    /**
     * Method to get the current heading of the gyro. Used for swerve
     * @return The Heading of the gyro in degrees.  
     */
    public double getHeading() {
        return Math.IEEEremainder(-navx.getAngle(), 360); 
    }

    /**
     * Method to get the rotation. 
     * @return Rotation2d in degrees. 
     */
    public Rotation2d getRotation2d() {
         return Rotation2d.fromDegrees(-navx.getYaw());

    }

    public Pose2d getPose() { 
        return odometer.getPoseMeters(); 
    }

    public void resetOdometry(Pose2d pose) { 
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    @Override
    public void periodic() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");

        //read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        // odometer.update(getRotation2d(), getModulePositions());
        // SmartDashboard.putNumber("Robot Heading", getRotation2d().getDegrees());
        // SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        // SmartDashboard.putNumber("Magnet", navx.getFusedHeading()); 
        // SmartDashboard.putNumber("LimelightX", x);
        // SmartDashboard.putNumber("LimelightY", y);
        // SmartDashboard.putNumber("LimelightArea", area);
        // SmartDashboard.putNumber("Rotation 2d", getRotation2d().getDegrees()); 
        // Encoder(); 
        field.setRobotPose(getPose());
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : SwerveMods){
             positions[mod.modNum] = mod.getPositions();
        }
        return positions;
    }

    
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] positions = new SwerveModuleState[4];
        for(SwerveModule mod : SwerveMods){
             positions[mod.modNum] = mod.getState();
        }
        return positions;
    }

    /**
     * Stop al wheels/modules when called. 
     */
    public void stopModules() {
        SwerveMods[0].stop();
        SwerveMods[1].stop();
        SwerveMods[2].stop();
        SwerveMods[3].stop();
    }

    /**
     * sets the state for each modules/wheel.
     * @param desiredStates Array of state. 
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        SwerveMods[0].setDesiredState(desiredStates[0], "Front Left");
        SwerveMods[1].setDesiredState(desiredStates[1], "Front Right");
        SwerveMods[2].setDesiredState(desiredStates[2], "Back Left");
        SwerveMods[3].setDesiredState(desiredStates[3], "Back Right");
    }

    public void Encoder() { 
        SmartDashboard.putNumber("Front Left", SwerveMods[0].getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Front RIght", SwerveMods[1].getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Back Left", SwerveMods[2].getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Back Right", SwerveMods[3].getAbsoluteEncoderRad());
    }

    public ChassisSpeeds getSpeeds() { 
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates()); 
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) { 
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02); 

        SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates); 
    }



    public void alignAprilTag() { 
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        double tableID = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0); 
        table.getEntry("pipeline").setNumber(tableID);
        NetworkTableEntry tx = table.getEntry("tx"); // Limelight's x-coordinate of the target
        NetworkTableEntry ty = table.getEntry("ty"); // Limelight's y-coordinate of the target
        NetworkTableEntry ta = table.getEntry("ta"); // Limelight's area of the target
        NetworkTableEntry tv = table.getEntry("tv");
        double prevarea = ta.getDouble(0); 
        double prevX = tx.getDouble(0); 
        double prevY = ty.getDouble(0); 
        // if(tableID == 6) 
        // SmartDashboard.putBoolean("ID DETECTED", true); 
        // else 
        // SmartDashboard.putBoolean("ID DETECTED", false); 
        
        boolean rotDone = false; 
        boolean xDone = false; 
        boolean yDone = false; 
        ChassisSpeeds chassisSpeeds; 

        // determine distance to goal
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);
        double mountAngleDegrees = 30.0;
        double lensHeight = 33.02;
        double goalHeight = 121.9;
        double angleToGoalRadians = (mountAngleDegrees + targetOffsetAngle_Vertical) * (Math.PI/180);
        double distLightToGoal = (goalHeight-lensHeight) / Math.tan(angleToGoalRadians);

    switch ((int) tableID) {
        case 6:
            while (true) { 
                SmartDashboard.putBoolean("ID DETECTED", true); 
                SmartDashboard.putBoolean("xDone", xDone);
                SmartDashboard.putBoolean("yDone", yDone);
                SmartDashboard.putBoolean("rotDone", rotDone);

                double y = ty.getDouble(0.0);
                double x = tx.getDouble(0.0);
                double area = ta.getDouble(0.0); 

                System.out.println(prevX);
                // Rotating Towards The Goal. 
                if ((getRotation2d().getDegrees() > 0.5 || getRotation2d().getDegrees() < -0.5) && !rotDone) { 
                    if (getRotation2d().getDegrees() > 0.5) { 
                        chassisSpeeds = new ChassisSpeeds(0, 0, -1); 
                        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                        setModuleStates(moduleStates);
                    } else if (getRotation2d().getDegrees() < -0.5) { 
                        chassisSpeeds = new ChassisSpeeds(0, 0, 1); 
                        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                        setModuleStates(moduleStates); 
                    }
                } else if (getRotation2d().getDegrees() < 0.5 && getRotation2d().getDegrees() > -0.5 && !rotDone || x == 0 ) { 
                    stopModules();
                    rotDone = true; 
                    break; 
                }


                // // Aligning X-Axis To The Goal
                // if ((prevX > 0.5 || prevX < -0.5 || x == 0) && !xDone && rotDone && !yDone) {
                //     if (x > 0.5 && !xDone) { 
                //         chassisSpeeds = new ChassisSpeeds(0, -.5, 0); 
                //         SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                //         setModuleStates(moduleStates);
                //     } 
                //      else if (x < -0.5 && !xDone) { 
                //         chassisSpeeds = new ChassisSpeeds(0, .5, 0); 
                //         SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                //         setModuleStates(moduleStates);
                //     }
                // } else if (x < 0.5 && x > -0.5 && !xDone && x != 0 && rotDone && !yDone) { 
                //     xDone = true; 
                //     stopModules();
                //     break;
                // }

                // // Aligning Y-Axis To The Goal And Exiting
                // if (y < 15 && xDone && rotDone && !yDone) { 
                //     chassisSpeeds = new ChassisSpeeds(.5, 0, 0); 
                //     SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                //     setModuleStates(moduleStates);
                // } else if (y >= 15 && xDone && rotDone && !yDone) { 
                //     yDone = true; 
                //     stopModules();
                //     break; 
                // }

// ==============================================================================================
            //     SmartDashboard.putBoolean("Y why", yDone); 
            //     if (area < 1000000 && area > 0.0) { 
    
            //         if (y > 14 && !yDone) { 
            //             if(getRotation2d().getDegrees() > 0.5) { 
            //                 chassisSpeeds = new ChassisSpeeds(0, 0, .5); 
            //                 SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            //                 setModuleStates(moduleStates);
            //             }
            //             else if (getRotation2d().getDegrees() < -0.5) { 
            //                 chassisSpeeds = new ChassisSpeeds(0, 0, -.5); 
            //                 SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            //                 setModuleStates(moduleStates);
            //             }
            //             else if (getRotation2d().getDegrees() > -.5 && getRotation2d().getDegrees() < .5) { 
            //                 stopModules();
            //                 prevarea = area; 
            //                 yDone = true;  
            //             }
            //         } else {
                    
            //         if (x > -0.5 && x < .5 && yDone) { 
            //             stopModules();  
            //             break; 
            //         }

            //         if (y < 14 && !yDone) { 
            //             chassisSpeeds = new ChassisSpeeds(.5, 0, 0); 
            //             SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            //             setModuleStates(moduleStates);

            //         } else if (x > .5) { 
            //             if (area >= 0.85) { 
            //                 stopModules();
            //                 break; 
            //             }
            //             chassisSpeeds = new ChassisSpeeds(0, -.5, 0); 
            //             SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            //             setModuleStates(moduleStates);

            //         } else if (x < -.5) { 
            //             if (area >= 0.85) { 
            //                 stopModules();
            //                 break; 
            //             }

            //             chassisSpeeds = new ChassisSpeeds(0, .5, 0); 
            //             SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            //             setModuleStates(moduleStates);
            //         }
            //     }
            //     } else { 
            //         stopModules();
            //         break; 
            //     }   
              }

        // break;
        
        default: 
            System.out.println("ASdasda");
                
        }
    }
 
}
 
