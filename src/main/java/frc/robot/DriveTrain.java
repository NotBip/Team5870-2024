package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
//import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class DriveTrain extends SubsystemBase{
    
    private static DifferentialDrive driveSystem = RobotMap.driveSystem;
   // The left-side drive encoder
    private final Encoder leftEncoder = RobotMap.leftEncoder; 
    // The right-side drive encoder
    private final Encoder rightEncoder = RobotMap.rightEncoder;
  private final  ADXRS450_Gyro m_gyro  = RobotMap.k_gyro;
    // The gyro sensor
    //private static final SPI.Port kGyroPort = SPI.Port.kOnboardCS0;
    
    
    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;
    
    /** Creates a new DriveSubsystem. */
    public DriveTrain() {
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        ///RobotMap.rightMotorGroup.setInverted(true);
        RobotMap.leftMotorGroup.setInverted(true);
        RobotMap.rightMotorGroup.setInverted(false);
        // Sets the distance per pulse for the encoders
        leftEncoder.setDistancePerPulse(RobotMap.kEncoderDistancePerPulse);
        rightEncoder.setDistancePerPulse(RobotMap.kEncoderDistancePerPulse);
         
        resetEncoders();
        m_gyro.reset();
        m_gyro.calibrate();
        m_odometry =
            new DifferentialDriveOdometry(
               m_gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
          
    }

    private void resetEncoders() {
        //reset position fo all encoders
        leftEncoder.reset(); rightEncoder.reset(); 
    }


    public void tankDriveVolts(double leftVolts, double rightVolts) {
        RobotMap.leftMotorGroup.setVoltage(leftVolts);
        RobotMap.rightMotorGroup.setVoltage(rightVolts);
        driveSystem.feed();
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
      return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
    }
  
    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public Encoder getLeftEncoder() {
      return leftEncoder;
    }
  
    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public Encoder getRightEncoder() {
      return rightEncoder;
    }
  
    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
  public void setMaxOutput(double maxOutput) {
      driveSystem.setMaxOutput(maxOutput);
    }
  
    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
      m_gyro.reset();
    }
  
    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
      return m_gyro.getRotation2d().getDegrees();
    }
  
    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
      return -m_gyro.getRate();
    }
    @Override
    public void periodic() {
      // Update the odometry in the periodic block
      m_odometry.update(
          m_gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
    }
  

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
      }
    
    public void resetOdometry(Pose2d pose) {
      resetEncoders();
      m_odometry.resetPosition(
          m_gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance(), pose);
    }
    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     * */

     public Pose2d getPose() {
        return m_odometry.getPoseMeters();
      }

    public void arcadeDrive(XboxController driverController){
      System.out.println(getAverageEncoderDistance());
        double forwardPower;
        double turnMod;
    
       
            turnMod = driverController.getLeftX();
            forwardPower = -driverController.getRightY();
        
    
        if (!driverController.getLeftBumper()) forwardPower *= 0.75;
    
        if (driverController.getRightBumper()) turnMod *= 0.5;
    
        driveSystem.arcadeDrive(turnMod, forwardPower);
    }

    public void arcadeDriveSet(double turnMod, double forwardPower){
      driveSystem.arcadeDrive(turnMod ,forwardPower);
    }
}  
