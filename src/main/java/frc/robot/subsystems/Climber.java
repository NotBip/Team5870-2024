package frc.robot.subsystems;
import frc.robot.Constants.climberConstants;
import frc.robot.Constants.climberConstants.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class Climber extends SubsystemBase {
    private CANSparkMax leaderMotor; 
    private CANSparkMax followerMotor; 
    DigitalInput bottomLimitSwitch;
    DigitalInput topLimitSwitch;
    private SparkPIDController m_pidController;
    private RelativeEncoder m_encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;



    public Climber() { 
        // Initializing motors and limit switches
        leaderMotor = new CANSparkMax(climberConstants.leaderMotor, MotorType.kBrushless);
        followerMotor = new CANSparkMax(climberConstants.followerMotor, MotorType.kBrushless);
        // bottomLimitSwitch = new DigitalInput(climberConstants.bottomLimitSwitch); 
        // topLimitSwitch = new DigitalInput(climberConstants.topLimitSwitch); 

        // Resetting motors
        leaderMotor.restoreFactoryDefaults();
        followerMotor.restoreFactoryDefaults();

        // Follower motor follows Leader Motor
        followerMotor.follow(leaderMotor);

        m_pidController = leaderMotor.getPIDController();
        m_encoder = leaderMotor.getEncoder();
                
        kP = 0.1; 
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = .5; 
        kMinOutput = -.5;

        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        // display PID coefficients on SmartDashboard
        // SmartDashboard.putNumber("P Gain", kP);
        // SmartDashboard.putNumber("I Gain", kI);
        // SmartDashboard.putNumber("D Gain", kD);
        // SmartDashboard.putNumber("I Zone", kIz);
        // SmartDashboard.putNumber("Feed Forward", kFF);
        // SmartDashboard.putNumber("Max Output", kMaxOutput);
        // SmartDashboard.putNumber("Min Output", kMinOutput);
        // SmartDashboard.putNumber("Set Rotations", 0);



    }

    public void moveArm(double motorSpeed) { 
        leaderMotor.set(motorSpeed);
    }

    public void setPosition(double rotations) { 
        m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    }

    
}
