package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;

public class SwerveDriveWheel {
    public PIDController directionController;
    public RelativeEncoder rotateEncoder; 
    public CANSparkMax driveMotors; 
    public CANSparkMax rotateMotors; 


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
            rotateMotors.setInverted(false);
            directionController.setSetpoint(currentAngle + setpointAngle);
        } else {
            // if the closest angle to setpoint + 180 is shorter
            // flip the motor direction and use the setpoint + 180
            rotateMotors.setInverted(true);
            directionController.setSetpoint(currentAngle + setpointAngleFlipped);
        }


        directionController.setSetpoint(currentAngle + closestAngle(currentAngle, setpoint));
    }

    public void setSpeed(double speed) {
        driveMotors.set(speed);
    }

    static double closestAngle(double a, double b) {
        // get direction
        double dir = (b % 360.0) - (a % 360.0);

        // convert from -360 to 360 to -180 to 180
        if (Math.abs(dir) > 180.0){
                dir = -(Math.signum(dir) * 360.0) + dir;
        }
        return dir;
    }
}   