package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.util.MotorUtils;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;

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
        MotorUtils.stopAllMotors();
    }

    public void swerveDrive(XboxController driverController) {
        if (driverController.getAButton()) System.out.println(driverController.getLeftX() + " " + driverController.getLeftY());
    }
}
