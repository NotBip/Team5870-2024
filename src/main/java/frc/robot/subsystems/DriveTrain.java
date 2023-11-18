package frc.robot.subsystems;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;

// import static frc.robot.util.MotorUtils.stopAllMotors;

/**
 * Auto-executing driving subsystem
 * asd
 * asd
 */

public class DriveTrain extends SubsystemBase {
    public DriveTrain() {
        Command justDrive = new RunCommand(() -> swerveDrive(Robot.io.driverController), this);
        setDefaultCommand(justDrive);
    }

    public void stop() {
        // stopAllMotors();
    }

    /**
     * Implicit drive using joystick
     * 
     * @param joy - driving joystick
     */
    public void swerveDrive(XboxController driverController) {

    }
}
