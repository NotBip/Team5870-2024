package frc.robot.util;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * @author Marston Connell <3
 * A togglable Double Solenoid wrapper
 */
public class DoubleSolenoidToggler extends DoubleSolenoid {

    public boolean extended = false;

    /**
     * @author Marston Connell <3
     * A togglable Double Solenoid wrapper
     */
    public DoubleSolenoidToggler(int portOne, int portTwo){
        super(PneumaticsModuleType.CTREPCM, portOne, portTwo);

    }

    /**
     * Toggles piston position.
     * @author Marston Connell <3
     */
    public void toggle(){
        //pushes out and turns extended to true
        if(!extended) {
            extend();
        //pulls in and turns extended to false
        } else {
            retract();
        }
    }

    /**
     * Sets piston to float and let air flow.
     * @author Marston Connell <3
     */
    public void setOff(){
        set(DoubleSolenoid.Value.kOff);
    }


    /**
     * Sets piston to extend.
     * @author Marston Connell
     */
    public void extend(){
        extended = true;
        set(DoubleSolenoid.Value.kForward);
    }

    /**
     * Sets piston to retract.
     * @author Marston Connell
     */
    public void retract(){
        extended = false;
        set(DoubleSolenoid.Value.kReverse);
    }


}