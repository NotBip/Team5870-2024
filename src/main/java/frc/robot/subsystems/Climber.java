package frc.robot.subsystems;

import frc.robot.Constants.climberConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Climber extends SubsystemBase {
    private CANSparkMax leaderMotor; 
    private CANSparkMax followerMotor; 
    // private CANSparkMax leftMotor; 
    // private CANSparkMax rightMotor; 
    DigitalInput bottomLimitSwitch;
    DigitalInput topLimitSwitch;
    private SparkPIDController m_pidController;
    private RelativeEncoder m_Encoder; 
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, kMotorSpeed, kRotations;



    public Climber() { 
        // Initializing motors and limit switches
        leaderMotor = new CANSparkMax(climberConstants.leaderMotor, MotorType.kBrushless);
        followerMotor = new CANSparkMax(climberConstants.followerMotor, MotorType.kBrushless);
        // leftMotor = new CANSparkMax(climberConstants.leaderMotor, MotorType.kBrushless);
        // rightMotor = new CANSparkMax(climberConstants.followerMotor, MotorType.kBrushless);

        // Resetting motors
        leaderMotor.restoreFactoryDefaults();
        followerMotor.restoreFactoryDefaults();

        // Follower motor follows Leader Motor
        followerMotor.follow(leaderMotor, true);        
        m_pidController = leaderMotor.getPIDController();
        m_Encoder = leaderMotor.getEncoder();
                
        kP = 0.05; 
        kD = 0;  
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = .5; 
        kMinOutput = -.5;

        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setD(kD);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);
        SmartDashboard.putNumber("Set Position", 0);
        SmartDashboard.putBoolean("Climber Spinning", false); 
    }

    @Override
    public void periodic() { 
        // double p = SmartDashboard.getNumber("P Gain", 0);
        // double d = SmartDashboard.getNumber("D Gain", 0);
        double motorSpeed = SmartDashboard.getNumber("Motor Speed", 0.20); 
        double rotations = SmartDashboard.getNumber("Set Position", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        SmartDashboard.putNumber("Climber Position", m_Encoder.getPosition()); 

        
        // if((p != kP)) { m_pidController.setP(p); kP = p; }
        // if((d != kD)) { m_pidController.setD(d); kD = d; }
        if((motorSpeed != kMotorSpeed)) { kMotorSpeed = motorSpeed; }
        if((rotations != kRotations)) { kRotations = rotations; } 
        if((max != kMaxOutput) || (min != kMinOutput)) { 
          m_pidController.setOutputRange(min, max); 
          kMinOutput = min; kMaxOutput = max; 
        }
    }

    public void moveLeft(double speed) { 
        // leftMotor.set(speed);
    }

    public void moveRight(double speed) { 
        // rightMotor.set(speed);
    }

    public void moveArm() { 
        leaderMotor.set(kMotorSpeed);
    }

    public void moveArmControllable(double speed) { 
        leaderMotor.set(speed);
    }

    public void setPosition(double setRotation) { 
        m_pidController.setReference(setRotation, CANSparkMax.ControlType.kPosition);
        SmartDashboard.putBoolean("Climber Spinning", true); 
    }

    public void hold() { 
        SmartDashboard.putBoolean("Climber Spinning", false); 
        leaderMotor.set(0);
        leaderMotor.setIdleMode(IdleMode.kBrake);   
        followerMotor.setIdleMode(IdleMode.kBrake); 
    }

    public void resetEncoders() { 
        m_Encoder.setPosition(0); 
        System.out.println("Climber Zeroed");
    }

    
}
