package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public class Shooter {

    private VictorSP shootMotor = new VictorSP(0);

    public Shooter() {}

    
    public void intake() {
        shootMotor.set(0.5);
    }

    public void stopMotor() {
        shootMotor.set(0.0);
    }

    public void shoot() {
        shootMotor.set(-0.5);
    }

}
