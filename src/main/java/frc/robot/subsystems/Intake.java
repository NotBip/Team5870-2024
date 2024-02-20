package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{

    private VictorSP shootMotor = new VictorSP(0);

    public Intake() {}

    
    public void moveSpin(double speed) {
        shootMotor.set(speed);
        SmartDashboard.putNumber("SPEEEED", speed);
    }


}
