package frc.robot.util;

import com.revrobotics.CANSparkMax;

import static frc.robot.RobotMap.ALL_MOTORS;

public class MotorUtils {
    public static void stopAllMotors() { for (CANSparkMax motor : ALL_MOTORS) motor.stopMotor(); }
}