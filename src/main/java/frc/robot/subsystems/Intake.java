package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * This class contains all the methods needed to run the intake subsystem. (MUST EXTEND SubsystemBase).  
 */
public class Intake extends SubsystemBase{

    // Initializing the Motors. 
    private VictorSP IntakeMotorFront = new VictorSP(IntakeConstants.armMotor1); // The front intake motor. 
    private VictorSP IntakeMotorBack = new VictorSP(IntakeConstants.armMotor2); // The back intake motor. 

    /**
     * This method sets the speed for the front motor by taking in a input. (THIS SUPPLIES THE INVERTED SPEED OF WHAT YOU INPUT) so it spins in the opposite direction of the back motors 
     * when given same input because of the way they were mounted lol. 
     * @param IntakeMotorFrontSpeed The speed at which the motor spins (-1 to 1)
     */
    public void IntakeMotorFront(double IntakeMotorFrontSpeed) { 
        IntakeMotorFront.set(-IntakeMotorFrontSpeed); //Invert the speed input to properly match with the back motor. 
    }


    /**
     * This method sets the speed for the back motor by taking in a input. (THIS ISN'T INVERTED)
     * @param IntakeMotorBackSpeed The speed at which the motor spins (-1 to 1)
     */
    public void IntakeMotorBack(double IntakeMotorBackSpeed) { 
        IntakeMotorBack.set(IntakeMotorBackSpeed);
    }

    /**
     * This method is used to completely stop both the front and back intake motors from spinning by setting their speeds to 0. 
     */
    public void intakeStop() { 
        IntakeMotorFront.set(0);
        IntakeMotorBack.set(0);
    }


}
