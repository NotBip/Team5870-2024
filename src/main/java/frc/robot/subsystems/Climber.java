package frc.robot.subsystems;

import frc.robot.Constants.climberConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;

/**
 * This class contains all the methods needed to run the Climber subsystem. (MUST EXTEND SubsystemBase).  
 */
public class Climber extends SubsystemBase {
    private CANSparkMax leaderMotor; // Define the leader motor. 
    private CANSparkMax followerMotor; // Define the follower motor. 
    private SparkPIDController m_pidController; // Define the PIDController to be able to make position setpoints for the climber. 
    private RelativeEncoder m_Encoder; // Define the encoder to get the current positon of the climber relative to it's starting position. 

    public Climber() { 
        // Initializing the CANSparkMax motors and their types. 
        leaderMotor = new CANSparkMax(climberConstants.leaderMotor, MotorType.kBrushless);
        followerMotor = new CANSparkMax(climberConstants.followerMotor, MotorType.kBrushless);

        // Resetting the motors
        leaderMotor.restoreFactoryDefaults();
        followerMotor.restoreFactoryDefaults();

        // Enabling the leader follower motor configuration and inverting the values for 1 of the motors because of the way they were mounted lol.
        followerMotor.follow(leaderMotor, true);        

        // Initializing the PIDController and the Encoder for the leader motor. 
        m_pidController = leaderMotor.getPIDController();
        m_Encoder = leaderMotor.getEncoder();
        
        // setting up the PID coefficients. 
        m_pidController.setP(.05);
        m_pidController.setD(0);
        m_pidController.setOutputRange(-.5, .5);
    }

    /**
     * This method sets the speed of the leader motor by taking in a double input. (follower motor naturally mirrors the input)
     * @param speed The speed at which the motor moves. (Recommened to have it below 0.4 to not break the climber we currently have).
     */
    public void moveArmControllable(double speed) { 
        leaderMotor.set(speed);
    }

    /**
     * This method takes in a setpoint and automatically positions the climber to match the setpoint. (The setpoint has to be set properly whenever the robot is turned on)
     * @param setRotation The target encoder positon of the motor. 
     */
    public void setPosition(double setRotation) { 
        m_pidController.setReference(setRotation, CANSparkMax.ControlType.kPosition); // Moves the motor to the position. 
    }

    /**
     * This method sets the speed for the leader motor to 0 and sets both the Leader and Follower motor to the brake idle mode. 
     */
    public void hold() { 
        leaderMotor.set(0);
        leaderMotor.setIdleMode(IdleMode.kBrake);   
        followerMotor.setIdleMode(IdleMode.kBrake); 
    }

    /**
     * Resets the leader motor encoder position. 
     */
    public void resetEncoders() { 
        m_Encoder.setPosition(0); 
        System.out.println("Climber Zeroed");
    }

    
}
