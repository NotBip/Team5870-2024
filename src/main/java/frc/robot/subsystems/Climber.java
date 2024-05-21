package frc.robot.subsystems;

import frc.robot.Constants.climberConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Climber extends SubsystemBase {
    private CANSparkMax leaderMotor; 
    private CANSparkMax followerMotor; 
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
        
        // PID and Output Values for the Climber Motors
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
