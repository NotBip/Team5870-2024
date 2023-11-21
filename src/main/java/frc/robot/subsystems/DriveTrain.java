package frc.robot.subsystems;

import frc.GodLIB.Controller;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotMap.Modules.TL;
import frc.robot.util.MotorUtils;
import frc.robot.util.SwerveDriveCoordinator;
import frc.robot.util.SwerveDriveWheel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import static frc.robot.RobotMap.Modules.*;
import static frc.GodLIB.Controller.getWithDeadzone;

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
        TL_TURN_WHEEL = new SwerveDriveWheel(TL.p, TL.i, TL.d, TL.rotateEncoder, TL.drive, TL.rotate);
        TR_TURN_WHEEL = new SwerveDriveWheel(TR.p, TR.i, TR.d, TR.rotateEncoder, TR.drive, TR.rotate);
        BL_TURN_WHEEL = new SwerveDriveWheel(BL.p, BL.i, BL.d, BL.rotateEncoder, BL.drive, BL.rotate);
        BR_TURN_WHEEL = new SwerveDriveWheel(BR.p, BR.i, BR.d, BR.rotateEncoder, BR.drive, BR.rotate);
        Wheels = new SwerveDriveCoordinator(TL_TURN_WHEEL, TR_TURN_WHEEL, BL_TURN_WHEEL, BR_TURN_WHEEL);
    }

    public void stop() {
        MotorUtils.stopAllMotors();
    }

    public void swerveDrive(Controller driverController) {
        // get joystick input
        // double angle = getAngle(driverController.getLeftX(), driverController.getLeftY());
        double angle = driverController.getAngle();
        double magnitude = getWithDeadzone(driverController.getStickSpeed(), RobotMap.LeftStickDeadBand);
        double twist = getWithDeadzone(driverController.getController().getRightX(), RobotMap.LeftStickDeadBand );
        SmartDashboard.putNumber("Left Stick", getWithDeadzone(driverController.getAngle(), RobotMap.LeftStickDeadBand));
        SmartDashboard.putNumber("Right Stick", getWithDeadzone(driverController.getAngle(true), RobotMap.RightStickDeadBand));



        // use field centric controls by subtracting off the robot angle
        angle -= RobotMap.GYRO.getAngle();

        Wheels.setSwerveDrive(angle, magnitude, twist);
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

        // if ((getPosition(driverController) > 0 ? getPosition(driverController) : -getPosition(driverController)) > LeftStickDeadBand) {
        //     Wheels.translate(getAngle(driverController), getPosition(driverController));
        //     System.out.println(getAngle(driverController));
        // }

        