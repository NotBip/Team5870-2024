package frc.GodLIB.Solenoid;

/**
 * Using two double solenoids to allow floating of piston.
 * @author Marston Connell
 */
public class FloatableSolenoid{

    private DoubleSolenoidToggler one;
    private DoubleSolenoidToggler two;
    private boolean extended = false;

    public FloatableSolenoid(int portOne, int portTwo, int portThree, int portFour) {
        one = new DoubleSolenoidToggler(portOne, portTwo);
        two = new DoubleSolenoidToggler(portThree, portFour);
    }

    public void floatPiston(){
        one.retract();
        two.retract();
    }

    public void retract() {
        two.retract();
        one.extend();
        extended = false;
    }

    public void extend() {
        one.retract();
        two.extend();
        extended = true;
    }

    public void toggle() {
        if (extended) {
            retract();
        } else {
            extend();
        }
    }

    public boolean isExtended() {
        return extended;
    }

}
