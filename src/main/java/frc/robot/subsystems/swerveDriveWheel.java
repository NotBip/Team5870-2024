package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.cutstomClasses.PIDOutput;
import frc.robot.cutstomClasses.PIDSource;


public class swerveDriveWheel {
 
    public PIDController directionController = new PIDController(0, 0, 0);
    public PIDOutput directionOutput;
    public PIDSource directionSource;

    public swerveDriveWheel() {
        
    }
}
