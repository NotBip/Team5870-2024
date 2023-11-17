package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.util.DoubleSolenoidToggler;
import frc.robot.util.FloatableSolenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.*;

/**
 * A system to grab and deliver hatch panels. Consists of two double solenoids: an extender and grabber.
 * 
 * @author Colin Toft
 * @author Rajan Sharma
 * @author Cam Hatherell
 * @author Marston Connell
 */
public class HatchDelivery extends SubsystemBase {
    
    private Compressor c;

    private FloatableSolenoid extenderSolenoid = RobotMap.extenderSolenoid;
    private DoubleSolenoidToggler grabberSolenoid = RobotMap.grabberSolenoid;

    private DigitalInput limitSwitch = new DigitalInput(RobotMap.HD_LIMIT_SWITCH_PORT);

    public HatchDelivery() {

        c = new Compressor(0, PneumaticsModuleType.CTREPCM);

        // auto compressor on when pressure low.
        c.enableDigital();
    }

    /** Toggles the grabber. If the grabber is closed, open it, if the grabber is open, close it. */
    public void toggleGrabber() {
        grabberSolenoid.toggle();
    }

    public void openGrabber() {
        grabberSolenoid.extend();
    }

    public void closeGrabber() {
        grabberSolenoid.retract();
    }

    /**
     * Turns on float mode.
     */
    public void floatExtender() {
        extenderSolenoid.floatPiston();
    }

    public boolean getLimitSwitch() {
        return limitSwitch.get();
    }

    public void extend() {
        extenderSolenoid.extend();
    }

    public void retract() {
        extenderSolenoid.retract();
    }
}
