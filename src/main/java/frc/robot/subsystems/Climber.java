package frc.robot.subsystems;

import frc.robot.Constants.climberConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Climber extends SubsystemBase {
    private Spark leaderMotor; 
    private Spark followerMotor; 
    DigitalInput bottomLimitSwitch;
    DigitalInput topLimitSwitch;
    private RelativeEncoder m_Encoder; 
    public double kMotorSpeed, kRotations;



    public Climber() { 
        // Initializing motors and limit switches
        leaderMotor = new Spark(climberConstants.leaderMotor);
        followerMotor = new Spark(climberConstants.followerMotor);

        // Resetting motors
        // leaderMotor.restoreFactoryDefaults();
        // followerMotor.restoreFactoryDefaults();

        // Follower motor follows Leader Motor
        followerMotor.setInverted(true);
        leaderMotor.addFollower(followerMotor);      
        // m_Encoder = leaderMotor.

        // set PID coefficients
        // m_pidController.setP(kP);
        // m_pidController.setD(kD);
        // m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        SmartDashboard.putNumber("Set Position", 0);

        // Instantiate a Spark MAX and get a handle to a PWM absolute encoder
        // CANSparkMax sparkmax = new CANSparkMax(0, MotorType.kBrushed);
        // var sparkencoder = sparkmax.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        // // Change this as needed, 8 points to average for velocity measurement is a
        // // decent starting point
        // sparkencoder.setAverageDepth(8);

        // // Reads the PWM Canandcoder position in rotations
        // sparkencoder.getPosition();

        // // configure the Spark Max to use the PWM-connected Canandcoder for
        // // closed-loop control
        // sparkmax.getPIDController().setFeedbackDevice(sparkencoder);

        // // Additionally, SparkMaxPIDController.setPositionPIDWrappingEnabled() and
        // // similar functions may be useful for swerve pivot applications.
    }

    @Override
    public void periodic() { 
        double motorSpeed = SmartDashboard.getNumber("Motor Speed", 0.20); 
        double rotations = SmartDashboard.getNumber("Set Position", 0);
        SmartDashboard.putNumber("Climber Position", m_Encoder.getPosition()); 

        if (motorSpeed != kMotorSpeed) { kMotorSpeed = motorSpeed; }
        if (rotations != kRotations) { kRotations = rotations; } 
    }

    public void moveArm() { 
        leaderMotor.set(kMotorSpeed);
    }

    public void moveArmControllable(double speed) { 
        leaderMotor.set(speed);
    }

    public void setPosition(double setRotation) { 
        m_Encoder.getPosition(); // - motor position in rotations
        // calculate required motor speed to get to desired position
    }

    public void hold() { 
        leaderMotor.set(0);
        leaderMotor.stopMotor(); 
        followerMotor.stopMotor(); 
    }

    public void resetEncoders() { 
        m_Encoder.setPosition(0); 
        System.out.println("Climber Zeroed");
    }

    
}
