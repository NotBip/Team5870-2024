package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SwerveDriveCoordinator {
    // Initializing all Wheels. 
    SwerveDriveWheel TLWheel;
    SwerveDriveWheel TRWheel;
    SwerveDriveWheel BLWheel;
    SwerveDriveWheel BRWheel;

    // Constructor for all Wheels. 
    public SwerveDriveCoordinator(SwerveDriveWheel TLWheele, SwerveDriveWheel TRWheele, SwerveDriveWheel BLWheele, SwerveDriveWheel BRWheele) {
        this.TLWheel = TLWheele;
        this.TRWheel = TRWheele;
        this.BLWheel = BLWheele;
        this.BRWheel = BRWheele;
    }

    // Method for Translate. 
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

    // Method for turning robot in Place. 
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

    // Method for turning robot while moving. 
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

    // Method to pick the correct method depending on controller input.
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
