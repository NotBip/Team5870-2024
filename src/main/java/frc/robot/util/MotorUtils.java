package frc.robot.util;

import com.revrobotics.CANSparkMax;

import static frc.robot.RobotMap.ALL_GROUP;;

public class MotorUtils {
    public static void stopAllMotors() { for (CANSparkMax motor : ALL_GROUP) motor.stopMotor(); }
}