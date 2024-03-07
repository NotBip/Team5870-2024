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
    DigitalInput bottomLimitSwitch;
    DigitalInput topLimitSwitch;
    private SparkPIDController m_pidController;
    private RelativeEncoder m_Encoder; 
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, kMotorSpeed, kRotations;



    public Climber() { 
        // Initializing motors and limit switches
        leaderMotor = new CANSparkMax(climberConstants.leaderMotor, MotorType.kBrushless);
        followerMotor = new CANSparkMax(climberConstants.followerMotor, MotorType.kBrushless);

        // Resetting motors
        leaderMotor.restoreFactoryDefaults();
        followerMotor.restoreFactoryDefaults();

        // Follower motor follows Leader Motor
        followerMotor.follow(leaderMotor, true);        
        m_pidController = leaderMotor.getPIDController();
        m_Encoder = leaderMotor.getEncoder();
                
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
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);
        SmartDashboard.putNumber("Set Position", 0);
    }

    @Override
    public void periodic() { 
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double motorSpeed = SmartDashboard.getNumber("Motor Speed", 0.20); 
        double rotations = SmartDashboard.getNumber("Set Position", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        get(); 

        
        if((p != kP)) { m_pidController.setP(p); kP = p; }
        if((i != kI)) { m_pidController.setI(i); kI = i; }
        if((d != kD)) { m_pidController.setD(d); kD = d; }
        if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
        if((motorSpeed != kMotorSpeed)) { kMotorSpeed = motorSpeed; }
        if((rotations != kRotations)) { kRotations = rotations; } 
        if((max != kMaxOutput) || (min != kMinOutput)) { 
          m_pidController.setOutputRange(min, max); 
          kMinOutput = min; kMaxOutput = max; 
        }
    }

    public void moveArm() { 
        leaderMotor.set(kMotorSpeed);
    }

    public void moveArmControllable(double speed) { 
        leaderMotor.set(speed);
    }

    public void setPosition() { 
        m_pidController.setReference(kRotations, CANSparkMax.ControlType.kPosition);
    }

    public void hold() { 
        leaderMotor.set(0);
        leaderMotor.setIdleMode(IdleMode.kBrake); 
        followerMotor.setIdleMode(IdleMode.kBrake); 
    }

    public void get() { 
        SmartDashboard.putNumber("Leader Motor Speed", leaderMotor.get()); 
    }

    public void resetEncoders() { 
        m_Encoder.setPosition(0); 
    }

    
}
