package frc.GodLIB;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
Quality of life functions for Xbox Controllers. 
@author Tallon Semeniuk
@version 1.0
@since September 21, 2023
*/

public class Controller {
    private XboxController controller;
    private CommandXboxController commandController;

    /**
    Constructor
    @param port The port of the Xbox Controller
    */
    public Controller(int port) {
        this.commandController = new CommandXboxController(port);
        this.controller = commandController.getHID();
    }

    
    /**
    Returns the Xbox Controller
    @return The Xbox Controller 
    */   
    public XboxController getController() {
        return controller; 
    }

    /**
    Method Used to get Angle's based on the position of Controller JoyStick.
    @param rightStick False, Null, or Blank would get LeftStick Angle, True would get RightStick Angle. 
    @return The current Angle of the Joystick in Degrees. 
    */
    public double getAngle(boolean... rightStick) {
        double y = rightStick.length > 0 && rightStick[0] ? controller.getRightY() : controller.getLeftY();
        double x = rightStick.length > 0 && rightStick[0] ? controller.getRightX() : controller.getLeftX();

        double Theta = Math.atan2(y, x);

        if (y < 0)
            Theta = Math.abs(Theta); 

        if (y > 0) {
            double something  = 180 - Math.toDegrees(Theta); 
            double something2 = 180 + something;
            return something2; 
        }
        
        return Math.toDegrees(Theta); 
    }
    
    /**
    Method used to Calculate speed based on JoyStick position can be Left or Right Stick. 
    @param rightStick False, Null, or Blank would get LeftStick Speed, True would get RightStick Speed. 
    @return The speed between 1.0 and -1.0 based on the JoyStick position. 
    */
    public double getStickSpeed(boolean... rightStick) {
        double y = rightStick.length > 0 && rightStick[0] ? controller.getRightY() : controller.getLeftY();
        double x = rightStick.length > 0 && rightStick[0] ? controller.getRightX() : controller.getLeftX();
        
        double pos = Math.sqrt(( y * y ) + ( x * x ));
        
        // remove if y-axis on school Controller has up as positive and down as negative.  
        pos = y > 0 ? -pos : pos; 
        
        // if speed is greater or less than 1 set it to 1. 
        if (pos > 1) pos = 1;
        else if (pos < -1) pos = -1;

        return pos; 
    }
    
    /**
    Method used to check if the input value passes the Deadzone check. 
    @param value Value that the Deadzone is being applied to. 
    @param deadzone Deadzone value to counter controller drift.  
    @return A double if the Deadzone check is passed. 
    */
    public static double getWithDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone) {
            return 0;
        }
        return value;
    }

    /**
    Run an instant command when a button is pressed.  
    @param button The button which runs the command. 
    @param onValue set to true or false depending on when you want the command to run. 
    @param command The InstantCommand you would like to run.   
    */
    public void registerButtonEvent(Button button, boolean onValue, InstantCommand command) {
        Trigger trigger = commandController.button(button.value);
        if (onValue) trigger.onTrue(command);
        else trigger.onFalse(command);
    }
    
    /**
    Run an repeat command when a button is pressed.  
    @param button The button which runs the command. 
    @param whileValue True if command runs when holding down button, false if otherwise. 
    @param command The RepeatCommand you would like to run.   
    */
    public void registerButtonEvent(Button button, boolean whileValue, RepeatCommand command) {
        Trigger trigger = commandController.button(button.value);
        if (whileValue) trigger.whileTrue(command);
        else trigger.whileFalse(command);
    }

    /**
    Instant Command to vibrate the Controller. Make sure you run endRumble() once your ready to end it
    @param power The intensity of the rumble. 
    @param left true if you want the Left Motor in the Controller to rumble.  
    @param right true if you want the Right Motor in the Controller to rumble.  
    */
    public void rumble(double power, boolean left, boolean right) {
        new InstantCommand(
            () -> {
                if (left) controller.setRumble(RumbleType.kLeftRumble, power);
                if (right) controller.setRumble(RumbleType.kRightRumble, power);
            }
	    ).initialize();
    }

    /**
    Instant Command to stop the controller vibrating 
    */
    public void endRumble() {
        new InstantCommand(
            () -> {
                controller.setRumble(RumbleType.kLeftRumble, 0.0);
                controller.setRumble(RumbleType.kRightRumble, 0.0);
            }
	    ).initialize();
    }
}
