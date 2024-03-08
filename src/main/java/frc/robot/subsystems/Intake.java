package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{

    private VictorSP IntakeMotorFront = new VictorSP(IntakeConstants.armMotor1);
    private VictorSP IntakeMotorBack = new VictorSP(IntakeConstants.armMotor2);
    // add shootmotor3
    
    public void moveSpin(double speed) {
        IntakeMotorFront.set(speed);
    }

    public void IntakeMotorFront(double IntakeMotorFrontSpeed) { 
        IntakeMotorFront.set(IntakeMotorFrontSpeed);

    }

    public void IntakeMotorBack(double IntakeMotorBackSpeed) 
    { 
        IntakeMotorBack.set(IntakeMotorBackSpeed);
    }

    public void intakeStop() { 
        IntakeMotorFront.set(0);
        IntakeMotorBack.set(0);
    }


}
