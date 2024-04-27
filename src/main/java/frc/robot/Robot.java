// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.security.PublicKey;
import java.sql.Driver;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot  {

    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer; 


    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        // Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

        // // if (isReal()) {
        // //     Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        // //     Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        // //     new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
        // // } else {
        //     setUseTiming(false); // Run as fast as possible
        //     String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        //     Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        //     Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        // // }

        // // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
        // Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

        m_robotContainer = new RobotContainer();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        // SmartDashboard.putNumber("ALLIANCE NUMBER", DriverStation.getLocation().getAsInt());
        // SmartDashboard.putString("ALLIANCE COLOR", DriverStation.getAlliance().get().toString());
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }

    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        // boolean autoDone = false; 
        // NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight"); 
        // double tableID = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0); 
        // NetworkTableEntry tx = table.getEntry("tx"); // Limelight's x-coordinate of the target
        // NetworkTableEntry ty = table.getEntry("ty"); // Limelight's y-coordinate of the target
        // NetworkTableEntry ta = table.getEntry("ta"); // Limelight's area of the target
        // double x = tx.getDouble(0); 
        // double y = ty.getDouble(0); 
        // double area = ta.getDouble(0); 
        // ChassisSpeeds chassisSpeeds; 
        // SmartDashboard.putBoolean("AUTO DONE", autoDone);


        // if (tableID == 6 && (m_robotContainer.swerveSubsystem.getHeading() <= -89 && m_robotContainer.swerveSubsystem.getHeading() >= -91) && !autoDone) { 
        //     m_autonomousCommand.cancel();;
        //     autoDone = true; 
        // }

        // if(tableID == 6) { 
        //     if (x < -2) { 
        //         SmartDashboard.putBoolean("ASDASDASDASD", true);
        //     }
        //     else { 
        //         SmartDashboard.putBoolean("ASDASDASDASD", false);

        //     }
        // }
    }

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
    public void testPeriodic() {
    }
}