package frc.GodLIB.Drive.Swerve;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
Library for Swerve Drive.
@author Hamad Mohammed, Tallon Semeniuk
@version 1.0
@since September 21, 2023
*/

public class SwerveDriveCoordinator {
    // Initializing all Wheels. 
    SwerveDriveWheel TLWheel;
    SwerveDriveWheel TRWheel;
    SwerveDriveWheel BLWheel;
    SwerveDriveWheel BRWheel;

    /**
    Constructor
    @param TLWheel Top Left wheel.
    @param TRWheel Top Bottom wheel.
    @param BLWheel Bottom Left wheel.
    @param BRWheel Bottom Right wheel.
    */
    public SwerveDriveCoordinator(SwerveDriveWheel TLWheel, SwerveDriveWheel TRWheel, SwerveDriveWheel BLWheel, SwerveDriveWheel BRWheel) {
        this.TLWheel = TLWheel;
        this.TRWheel = TRWheel;
        this.BLWheel = BLWheel;
        this.BRWheel = BRWheel;
    }

    
    /**
    Method for translating the robot where all the wheels face the same direction and move in the same direction (Make sure to call it only when input from left JoyStick). 
    @param direction Wheel direction you would like to set in degrees. 
    @param power Speed of the driving wheels you would like (1.0 to -1.0)
    */
    public void translate(double direction, double power) {
        TLWheel.setDirection(direction);
        TRWheel.setDirection(direction);
        BLWheel.setDirection(direction);
        BRWheel.setDirection(direction);
        
        TLWheel.setSpeed(power);
        TRWheel.setSpeed(power);
        BLWheel.setSpeed(power);
        BRWheel.setSpeed(power);
    }

    /**
    Method for turning the robot in place (Make sure to only call when input from right JoyStick). 
    @param power Speed of the driving wheels you would like (1.0 to -1.0)
    */
    public void inplaceTurn(double power) {
        TLWheel.setDirection(45.0);
        TRWheel.setDirection(-45.0);
        BLWheel.setDirection(135.0);
        BRWheel.setDirection(-135.0);

        TLWheel.setSpeed(power);
        TRWheel.setSpeed(power);
        BLWheel.setSpeed(power);
        BRWheel.setSpeed(power);
    }

    /**
    Method for turning the robot while moving (Call when input from both JoySticks). 
    @param direction  Direction of the wheels. 
    @param translatePower Speed of the driving wheels you would like (1.0 to -1.0)/
    @param turnPower Gets turn speed from (usually) right JoyStick to calculate turnAngle. 
    */
    public void translateTurn(double direction, double translatePower, double turnPower) {
        double turnAngle = turnPower * 45.0;

        // if the left front wheel is in the front
        if (SwerveDriveWheel.closestAngle(direction, 135.0) >= 90.0) {
            TLWheel.setDirection(direction + turnAngle);
        } else {
            // if it's in the back
            TLWheel.setDirection(direction - turnAngle);
        }
        // if the left back wheel is in the front
        if (SwerveDriveWheel.closestAngle(direction, 225.0) > 90.0) {
            BLWheel.setDirection(direction + turnAngle);
        } else {
            // if it's in the back
            BLWheel.setDirection(direction - turnAngle);
        }
        // if the right front wheel is in the front
        if (SwerveDriveWheel.closestAngle(direction, 45.0) > 90.0) {
            TRWheel.setDirection(direction + turnAngle);
        } else {
            // if it's in the back
            TRWheel.setDirection(direction - turnAngle);
        }
        // if the right back wheel is in the front
        if (SwerveDriveWheel.closestAngle(direction, 315.0) >= 90.0) {
            BRWheel.setDirection(direction + turnAngle);
        } else {
            // if it's in the back
            BRWheel.setDirection(direction - turnAngle);
        }
        
        TLWheel.setSpeed(translatePower);
        TRWheel.setSpeed(translatePower);
        BLWheel.setSpeed(translatePower);
        BRWheel.setSpeed(translatePower);
    }

    /**
    Method to have the robot actually move based on which joyStick you are getting input from.  
    @param direction  Direction of the wheels. 
    @param translatePower Speed of the driving wheels you would like (1.0 to -1.0)/
    @param turnPower Gets turn speed from (usually) right JoyStick to calculate turnAngle. 
    */
    public void setSwerveDrive(double direction, double translatePower, double turnPower) {
        if ((translatePower == 0.0) && (turnPower != 0.0)) {
            SmartDashboard.putString("Type", "Inplace");
            inplaceTurn(turnPower);
        } else if ((translatePower != 0.0) && (turnPower != 0.0)) {
            SmartDashboard.putString("Type", "Turn While Moving");
            translateTurn(direction, translatePower, turnPower);
        } else if ((translatePower != 0.0 ) && (turnPower == 0.0)){
            SmartDashboard.putString("Type", "Translate");
            translate(direction, translatePower);
        } else {
            SmartDashboard.putString("Type", "Idle");
            TLWheel.setSpeed(translatePower);
            TRWheel.setSpeed(translatePower);
            BLWheel.setSpeed(translatePower);
            BRWheel.setSpeed(translatePower);
        }

        SmartDashboard.putNumber("Direction", direction);
        SmartDashboard.putNumber("Turn Power", turnPower);
        SmartDashboard.putNumber("Top Left Wheel", TLWheel.driveMotors.get());
        SmartDashboard.putNumber("Top Right Wheel", TRWheel.driveMotors.get());
        SmartDashboard.putNumber("Bottom Left Wheel", BLWheel.driveMotors.get());
        SmartDashboard.putNumber("Bottom Right Wheel", BRWheel.driveMotors.get());
    }
}
