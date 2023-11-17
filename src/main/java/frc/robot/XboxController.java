package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class XboxController extends Joystick{
	
	public XboxController(int port) {
		super(port);
	}
	
	public void setLeftRumble(double rumble) {
		setRumble(Joystick.RumbleType.kLeftRumble, (float)rumble);
	}
	
	public void setRightRumble(double rumble) {
		setRumble(Joystick.RumbleType.kRightRumble, (float)rumble);
	}
	
	public double getLeftStickXAxis() {
		return getRawAxis(0);
	}
	
	public double getLeftStickYAxis() {
		return -getRawAxis(1);
	}
	
	public double getRightTriggerAxis() {
		return getRawAxis(3);
	}
	
	public double getLeftTriggerAxis() {
		return getRawAxis(2);
	}
	
	public double getRightStickXAxis() {
		return getRawAxis(4);
	}
	
	public double getRightStickYAxis() {
		return -getRawAxis(5);
	}

	public boolean getAButton() {
		return getRawButton(1);
	}

	public boolean getBButton() {
		return getRawButton(2);
	}

	public boolean getXButton() {
		return getRawButton(3);
	}

	public boolean getYButton() {
		return getRawButton(4);
	}

	public boolean getLeftBumper() {
		return getRawButton(5);
	}

	public boolean getRightBumper() {
		return getRawButton(6);
	}

	public boolean getBackButton() {
		return getRawButton(7);
	}

	public boolean getStartButton() {
		return getRawButton(8);
	}
	
	public boolean getLeftJoystickButton() {
		return getRawButton(9);
	}
	
	public boolean getRightJoystickButton() {
		return getRawButton(10);
	}
	
	public boolean getRightTriggerButton() {
		if(getRightTriggerAxis() > 0.5) {
			return true;
		} else {
			return false;
		}
	}
	
	public boolean getLeftTriggerButton() {
		if(getLeftTriggerAxis() > 0.5) {
			return true;
		} else {
			
			return false;
		}
	}
	
	public int getDPad() {
		return getPOV();
	}
	
	public boolean isDPadTopHalf() {
		if(getDPad() == 315 || getDPad() == 0 || getDPad() == 45) {
			return true;
		} else {
			return false;
		}
	}
	
	public boolean isDPadBottomHalf() {
		if(getDPad() == 135 || getDPad() == 180 || getDPad() == 225) {
			return true;
		} else {
			return false;
		}
	}
	
	public double getBothTriggerAxis() {
		return (getRightTriggerAxis()-getLeftTriggerAxis());
	}
	
	public double getTriggers() {
		return (getRawAxis(3)-getRawAxis(2));
	}
	
}