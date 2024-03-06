package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{

    private VictorSP shootMotor = new VictorSP(IntakeConstants.armMotor);
    private double motorSpeed; 

    public Intake() {}

    
    public void moveSpin(double speed) {
        motorSpeed = speed; 
        shootMotor.set(speed);
        SmartDashboard.putNumber("Intake Speed", speed); 
    }

    public void periodic() { 
        SmartDashboard.putNumber("Intake Speed", motorSpeed); 
    }

}
