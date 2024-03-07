package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{

    private VictorSP IndividualMotorAtPortNo9 = new VictorSP(IntakeConstants.armMotor1);
    private VictorSP IndividualMotorAtPortNo8 = new VictorSP(IntakeConstants.armMotor2);
    // add shootmotor3
    
    public void moveSpin(double speed) {
        IndividualMotorAtPortNo9.set(speed);
    }

    public void moveSpinIndividualMotorAtPort9(double SpeedForIndividualMotorForPort9) { 
        IndividualMotorAtPortNo9.set(SpeedForIndividualMotorForPort9);
        SmartDashboard.putNumber("Intake Speed For Individual Motor For Port 9 On Roborio 2.0 On 2024 Robot (Sia) For Crescendodododod with team 5870 by hamad on asus tuf gaming f15 (not sponsored) laptop with 144hz monitor with adaptive sync. This was written using the laptop on the asus tuf gaming f15 laptop keyboard", SpeedForIndividualMotorForPort9); 

    }

    public void moveSpinIndividualMotorAtPort8(double SpeedForIndividualMotorForPort8) 
    { 
        IndividualMotorAtPortNo8.set(SpeedForIndividualMotorForPort8);
        SmartDashboard.putNumber("Intake Speed For Individual Motor For Port 8 On Roborio 2.0 On 2024 Robot (Sia) For Crescendodododod with team 5870 by hamad on asus tuf gaming f15 (not sponsored) laptop with 144hz monitor with adaptive sync. This was written using the laptop on the asus tuf gaming f15 laptop keyboard", SpeedForIndividualMotorForPort8); 
    }

    public void intakeStop() { 
        IndividualMotorAtPortNo8.set(0);
        IndividualMotorAtPortNo9.set(0);
    }


}
