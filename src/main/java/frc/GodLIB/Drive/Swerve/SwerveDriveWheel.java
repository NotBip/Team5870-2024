package frc.GodLIB.Drive.Swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
Library for Swerve Drive.
@author Hamad Mohammed, Tallon Semeniuk
@version 1.0
@since September 21, 2023
*/

public class SwerveDriveWheel {
    public PIDController directionController;
    public RelativeEncoder rotateEncoder; 
    public CANSparkMax driveMotors; 
    public CANSparkMax rotateMotors; 

    /**
    Constructor
    @param P Proportional Gain
    @param I Integral Gain
    @param D Derivitave Gain
    @param encoder Relative Encoder to the rotating motor of the wheel
    @param drive The Drive motor
    @param rotate The Rotate motor
    */
    public SwerveDriveWheel(double P, double I, double D, RelativeEncoder encoder, CANSparkMax drive, CANSparkMax rotate) {
        this.rotateEncoder = encoder; 
        directionController = new PIDController(P, I, D);
        this.driveMotors = drive; 
        this.rotateMotors = rotate;
    }

    
    public void setDirection(double setpoint) {
        directionController.reset();
        double currentAngle = rotateEncoder.getPosition();
        double setpointAngle = closestAngle(currentAngle, setpoint);

        double setpointAngleFlipped = closestAngle(currentAngle, setpoint + 180.0);

        if (Math.abs(setpointAngle) <= Math.abs(setpointAngleFlipped)) {
            // unflip the motor direction use the setpoint
            driveMotors.setInverted(false);
            SmartDashboard.putNumber("Wheel Point: ", (currentAngle + setpointAngle));
            directionController.setSetpoint(currentAngle + setpointAngle);
            driveMotors.set(Math.abs(driveMotors.get()));
            SmartDashboard.putBoolean("Inverted", false); 
        } else {    
            // if the closest angle to setpoint + 180 is shorter
            // flip the motor direction and use the setpoint + 180
            driveMotors.set(-driveMotors.get());
            SmartDashboard.putNumber("Wheel Point Flipped: ", (currentAngle + setpointAngleFlipped));
            directionController.setSetpoint(currentAngle + setpointAngleFlipped);
            SmartDashboard.putBoolean("Inverted", true); 
        }    
    }
    

    public void setSpeed(double speed) {
        driveMotors.set(speed);
    }

    // Find the Closing angle to get to position wanted.
    static double closestAngle(double a, double b) {
        // get direction
        double dir = (b % 360.0) - (a % 360.0);

        // convert from -360 to 360 to -180 to 180
        if (Math.abs(dir) > 180.0) {
            dir = -(Math.signum(dir) * 360.0) + dir;
        }
        
        return dir;
    }
}   