package frc.robot.subsystems;

import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.*;

/**
 * Auto-executing driving subsystem
 * asd
 * asd
 */

public class DriveTrain extends SubsystemBase {
    private DifferentialDrive driveSystem = RobotMap.driveSystem;

    public DriveTrain() {


    }

    public void stop() {
        driveSystem.stopMotor();
    }


    /**
     * Implicit drive using joystick
     * 
     * @param joy - driving joystick
     */
    public void arcadeDrive(XboxController driverController) {

        /** If enabled, right trigger moves the robot forwards and left trigger moves the robot backwards. */
        boolean octaneDrive = true; // marston was here
        
        double forwardPower;
        double turnMod;

        if (octaneDrive) {
            forwardPower = -(driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis());
            turnMod = driverController.getRightX();
        } else {
            forwardPower = -driverController.getLeftY();
            turnMod = driverController.getRightX();
        }

        if (!driverController.getRightBumper()) forwardPower *= 0.75;

        if (driverController.getLeftBumper()) turnMod *= 0.5;

        driveSystem.arcadeDrive(forwardPower, turnMod);
    }

        /**
     * Implicit drive using joystick
     * 
     * @param joy - driving joystick
     */
    public void swerveDrive(XboxController driverController) {

    }

    /**
     * Log important data
     */
    public void log() {
        
    }

    public double getAngle(int encoderTicks){
        
        return (encoderTicks % 1440) / 1440 * 30; 
    }
}
