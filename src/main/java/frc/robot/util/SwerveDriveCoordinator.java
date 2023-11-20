package frc.robot.util;


public class SwerveDriveCoordinator {
    SwerveDriveWheel TLWheel;
    SwerveDriveWheel TRWheel;
    SwerveDriveWheel BLWheel;
    SwerveDriveWheel BRWheel;

    public SwerveDriveCoordinator(SwerveDriveWheel TLWheele, SwerveDriveWheel TRWheele, SwerveDriveWheel BLWheele, SwerveDriveWheel BRWheele) {
        this.TLWheel = TLWheele;
        this.TRWheel = TRWheele;
        this.BLWheel = BLWheele;
        this.BRWheel = BRWheele;
    }

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

    public void setSwerveDrive(double direction, double translatePower, double turnPower) { // never used, whats the point of this?
        if ((translatePower == 0.0) && (turnPower != 0.0)) {
            inplaceTurn(turnPower);
        } else {
            translateTurn(direction, translatePower, turnPower);
        }
    }
}
