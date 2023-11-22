package frc.GodLIB.Drive.Arcade;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.GodLIB.Controller;

/**
Simplifying Differential Drives. 
@author Tallon Semeniuk
@version 1.0
@since September 21, 2023
*/
public class ArcadeDrive extends SubsystemBase {
    private DifferentialDrive driveSystem;

    /**
    Constructor
    @param controller set Controller. 
    @param leftMotorGroup Motor group for top and bottom left motors. 
    @param rightMotorGroup  Motor group for top and bottom right motors. 
    */
    public ArcadeDrive(Controller controller, MotorControllerGroup leftMotorGroup, MotorControllerGroup rightMotorGroup) {
        driveSystem = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

        Command justDrive = new RunCommand(() -> arcadeDrive(controller), this);
        setDefaultCommand(justDrive);
        // Naming sensors for live window
        addChild("Drive", driveSystem);
        driveSystem.setDeadband(0.1);
    }

    /**
    Method for controlling a robot with differential Drive
    @param controller set Controller. 
    */
    public void arcadeDrive(Controller controller) {
        /** If enabled, right trigger moves the robot forwards and left trigger moves the robot backwards. */
        boolean octaneDrive = false; // marston was here
        
        double forwardPower;
        double turnMod;

        if (octaneDrive) {
            forwardPower = -(controller.getController().getRightTriggerAxis() - controller.getController().getLeftTriggerAxis());
            turnMod = controller.getController().getRightX();
        } else {
            forwardPower = -controller.getController().getLeftY();
            turnMod = controller.getController().getLeftX();
        }

        if (!controller.getController().getRightBumper()) forwardPower *= 0.75;

        if (controller.getController().getLeftBumper()) turnMod *= 0.5;

        driveSystem.arcadeDrive(forwardPower, turnMod);
    }
}
