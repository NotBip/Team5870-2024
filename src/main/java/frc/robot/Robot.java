// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    // private SwerveSubsystem swerveSubsystem;
    // private Double  xSpdFunction, ySpdFunction, turningSpdFunction;
    // private Boolean fieldOrientedFunction;
    // private SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  // public boolean asd; 
  //  private VictorSP motor; 

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
     m_robotContainer = new RobotContainer();
     
    
    //     this.swerveSubsystem = m_robotContainer.swerveSubsystem;
    //     this.xSpdFunction = m_robotContainer.driverJoytick.getRawAxis(OIConstants.kDriverYAxis);
    //     this.ySpdFunction = -m_robotContainer.driverJoytick.getRawAxis(OIConstants.kDriverXAxis);
    //     this.turningSpdFunction = m_robotContainer.driverJoytick.getRawAxis(OIConstants.kDriverRotAxis);
    //     this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    //     this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    //     this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    //  motor = new VictorSP(6); 
   // asd = m_robotContainer.button(); 
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  //OIConstants.kDeadband ? turningSpeed : 0.0
  // if (m_robotContainer.getRotAxis() > OIConstants.kDeadband || -m_robotContainer.getRotAxis() > OIConstants.kDeadband){
  //   m_robotContainer.getSwerveSubsystem().frontLeft.offset = ((Math.PI)/2); 
  //   m_robotContainer.getSwerveSubsystem().frontRight.offset = -((Math.PI)/2); 
  //   m_robotContainer.getSwerveSubsystem().backLeft.offset = -((Math.PI)/2); 
  //   m_robotContainer.getSwerveSubsystem().backRight.offset = ((Math.PI)/2); 
  // }
  // if (m_robotContainer.getRotAxis() < OIConstants.kDeadband) {
  //   System.out.println("less"); 
  //   m_robotContainer.getSwerveSubsystem().frontLeft.offset = 0; 
  //   m_robotContainer.getSwerveSubsystem().frontRight.offset = 0; 
  //   m_robotContainer.getSwerveSubsystem().backLeft.offset = 0; 
  //   m_robotContainer.getSwerveSubsystem().backRight.offset = 0; 
  // }

   
   
   
    // // 1. Get real-time joystick inputs
    //     double xSpeed = xSpdFunction;
    //     double ySpeed = ySpdFunction;
    //     double turningSpeed = turningSpdFunction;

    //     // 2. Apply deadband
    //     xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    //     ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
    //     turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

    //     // 3. Make the driving smoother
    //     xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    //     ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    //     turningSpeed = turningLimiter.calculate(turningSpeed)
    //             * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    //     // 4. Construct desired chassis speeds
    //     ChassisSpeeds chassisSpeeds;
    //     // if (fieldOrientedFunction.get()) {
    //     //     // Relative to field
    //     //     chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
    //     //             xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
    //   //  } else {
    //         // Relative to robot
    //         chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    //    // }

    //     // 5. Convert chassis speeds to individual module states
    //     SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    

    //     // 6. Output each module states to wheels
    //     swerveSubsystem.setModuleStates(moduleStates);
    //     swerveSubsystem.Encoder();
   
   
   
    //  motor.set(-m_robotContainer.driverController.getRightTriggerAxis());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
