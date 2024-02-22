package frc.robot.subsystems;
import frc.robot.Constants.climberConstants;
import frc.robot.Constants.climberConstants.*;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class Climber extends SubsystemBase {
    private CANSparkMax leaderMotor; 
    private CANSparkMax followerMotor; 
    DigitalInput bottomLimitSwitch;
    DigitalInput topLimitSwitch;


    public Climber() { 
        // Initializing motors and limit switches
        leaderMotor = new CANSparkMax(climberConstants.leaderMotor, MotorType.kBrushless);
        followerMotor = new CANSparkMax(climberConstants.followerMotor, MotorType.kBrushless);
        bottomLimitSwitch = new DigitalInput(climberConstants.bottomLimitSwitch); 
        topLimitSwitch = new DigitalInput(climberConstants.topLimitSwitch); 

        // Resetting motors
        leaderMotor.restoreFactoryDefaults();
        followerMotor.restoreFactoryDefaults();

        // Follower motor follows Leader Motor
        followerMotor.follow(leaderMotor);
    }

    public void moveArm(double motorSpeed) { 
        leaderMotor.set(motorSpeed);
    }

    
}
