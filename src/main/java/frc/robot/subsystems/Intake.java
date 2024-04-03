package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{

    private VictorSP IntakeMotorFront = new VictorSP(IntakeConstants.armMotor1);
    private VictorSP IntakeMotorBack = new VictorSP(IntakeConstants.armMotor2);
    // add shootmotor3
    
    public Intake() { 
        SmartDashboard.putBoolean("Intake Spinning", false);
    }

    public void moveSpin(double speed) {
        IntakeMotorFront.set(speed);
        SmartDashboard.putBoolean("Intake Spinning", true);
    }

    public void IntakeMotorFront(double IntakeMotorFrontSpeed) { 
        IntakeMotorFront.set(-IntakeMotorFrontSpeed);
        SmartDashboard.putBoolean("Intake Spinning", true);

    }

    public void IntakeMotorBack(double IntakeMotorBackSpeed) { 
        IntakeMotorBack.set(IntakeMotorBackSpeed);
        SmartDashboard.putBoolean("Intake Spinning", true);
    }

    public void intakeStop() { 
        IntakeMotorFront.set(0);
        IntakeMotorBack.set(0);
        SmartDashboard.putBoolean("Intake Spinning", false);
    }


}
