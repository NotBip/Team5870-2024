package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

/**
 * This class contains all the methods needed for the pneumatics subsystem. (MUST EXTEND SubsystemBase).
 */
public class Pneumatics extends SubsystemBase{
    
    private Compressor c; // Define the Compressor. (Is responsible for storing up air to then later supply to the pistons)
    private Solenoid solenoidExtend1; // Defining the first solenoid used for extending
    private Solenoid solenoidExtend2; // Defining the second solenoid used for extending
    private Solenoid solenoidDetract1; // Defining the first solenoid used for detracting
    private Solenoid solenoidDetract2; // Defining the second solenoid used for detracting

    public Pneumatics() { 
        c = new Compressor(0, PneumaticsModuleType.CTREPCM); // Initalizing the compressor. 

        // Initializing the Solenoids with their ID's and types. 
        solenoidExtend1 = new Solenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.solenoidExtend1ID); 
        solenoidExtend2 = new Solenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.solenoidExtend2ID); 
        solenoidDetract1 = new Solenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.solenoidDetract1ID); 
        solenoidDetract2 = new Solenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.solenoidDetract2ID); 
     
        c.enableDigital(); // Enable the compressor at the start of the robot when this class is initialized. 
    }

    /**
     * This method is used to extend the pneumatic pistons.  
     */
    public void extendArm() { 

        // Turn Extender Solenoids on
        solenoidExtend1.set(true);
        solenoidExtend2.set(true);

        // Turn Detract Solenoids off
        solenoidDetract1.set(false);
        solenoidDetract2.set(false);
    }

    /**
     * This method is used to detract the pneumatic pistons back. 
     */
    public void detractArm() { 

        // Turn Extender Solenoids off
        solenoidExtend1.set(false);
        solenoidExtend2.set(false);

        // Turn Detract Solenoids on
        solenoidDetract1.set(true);
        solenoidDetract2.set(true);
    }

    
    
    

}
