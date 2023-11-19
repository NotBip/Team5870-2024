package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap.Modules.TL;
import frc.robot.util.MotorUtils;
import frc.robot.util.SwerveDriveCoordinator;
import frc.robot.util.SwerveDriveWheel;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import static frc.robot.RobotMap.LeftStickDeadBand;
import static frc.robot.RobotMap.Modules.*;

/**
 * Auto-executing driving subsystem
 * asd
 * asd
 */

public class DriveTrain extends SubsystemBase {
    SwerveDriveWheel TL_TURN_WHEEL; 
    SwerveDriveWheel TR_TURN_WHEEL; 
    SwerveDriveWheel BL_TURN_WHEEL; 
    SwerveDriveWheel BR_TURN_WHEEL; 
    SwerveDriveCoordinator Wheels;


    public DriveTrain() {
        Command justDrive = new RunCommand(() -> swerveDrive(Robot.io.driverController), this);
        setDefaultCommand(justDrive);
        TL_TURN_WHEEL = new SwerveDriveWheel(TL.p, TL.i, TL.d, TL.rotateEncoder, TL.drive);
        TR_TURN_WHEEL = new SwerveDriveWheel(TR.p, TR.i, TR.d, TR.rotateEncoder, TR.drive);
        BL_TURN_WHEEL = new SwerveDriveWheel(BL.p, BL.i, BL.d, BL.rotateEncoder, BL.drive);
        BR_TURN_WHEEL = new SwerveDriveWheel(BR.p, BR.i, BR.d, BR.rotateEncoder, BR.drive);
        Wheels = new SwerveDriveCoordinator(TL_TURN_WHEEL, TR_TURN_WHEEL, BL_TURN_WHEEL, BR_TURN_WHEEL);
    }

    public void stop() {
        MotorUtils.stopAllMotors();
    }

    public void swerveDrive(XboxController driverController) {
        if ((getPosition(driverController) > 0 ? getPosition(driverController) : -getPosition(driverController)) > LeftStickDeadBand) {
            Wheels.translate(getAngle(driverController), getPosition(driverController));
            System.out.println(TL_TURN_WHEEL.driveMotors.get());
        }
    }

    public double getPosition(XboxController driverController) {
        double pos = Math.sqrt(((driverController.getRightX() * driverController.getRightX()) + (driverController.getRightY() * driverController.getRightY())));
        pos = driverController.getRightY() > 0 ? pos*-1 : pos;
        if (pos > 1) pos = 1;
        else if (pos < -1) pos = -1;
        return pos; 
    }

    public double getAngle(XboxController driverController) {
        double Theta = Math.atan((double) driverController.getRightY() / (double) driverController.getRightX());
        return Math.toDegrees(Theta); 
    }
}
        // double Theta = Math.atan(driverController.getRightY() / driverController.getRightX());

        // top left   -1, -1
        // top right   1, -1
        // bottom left -1, 1
        // buttom right 1, 1

        // top left 45
        // top right -45
        // bottom left -45
        // bottom right 45



        