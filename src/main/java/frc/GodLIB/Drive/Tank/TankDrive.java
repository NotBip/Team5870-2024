package frc.GodLIB.Drive.Tank;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.GodLIB.Controller;

/**
Simplifying Tank Drives. 
@author Tallon Semeniuk
@version 1.0
@since September 21, 2023
*/
public class TankDrive extends SubsystemBase {
    private DifferentialDrive driveSystem;

    /**
    Constructor
    @param controller set Controller. 
    @param leftMotorGroup Motor group for top and bottom left motors. 
    @param rightMotorGroup  Motor group for top and bottom right motors. 
    */
    public TankDrive(Controller controller, MotorControllerGroup leftMotorGroup, MotorControllerGroup rightMotorGroup) {
        driveSystem = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

        Command justDrive = new RunCommand(() -> tankDrive(controller), this);
        setDefaultCommand(justDrive);
        // Naming sensors for live window
        addChild("Drive", driveSystem);
        driveSystem.setDeadband(0.1);
    }

    /**
    Method for controlling a robot with Tank Drive
    @param driverController set Controller. 
    */
    public void tankDrive(Controller controller) {
        double left = controller.getController().getLeftY();
        double right = controller.getController().getLeftY();
        driveSystem.tankDrive(left, right);
    }
}
