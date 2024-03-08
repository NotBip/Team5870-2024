package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class Pneumatics extends SubsystemBase{
    
    private Compressor c; 
    boolean CompressorOn = false; 
    private Solenoid solenoidExtend1;
    private Solenoid solenoidExtend2;
    private Solenoid solenoidDetract1; 
    private Solenoid solenoidDetract2; 

    public Pneumatics() { 
        c = new Compressor(0, PneumaticsModuleType.CTREPCM); 
        solenoidExtend1 = new Solenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.solenoidExtend1ID); 
        solenoidExtend2 = new Solenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.solenoidExtend2ID); 
        solenoidDetract1 = new Solenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.solenoidDetract1ID); 
        solenoidDetract2 = new Solenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.solenoidDetract2ID); 
     
        startCompressor();
    }

    public void startCompressor() { 
        // Start Compressor
        c.enableDigital(); 
    }

    public void extendArm() { 
        // Turn Extender Solenoids on
        solenoidExtend1.set(true);
        solenoidExtend2.set(true);
        // Turn Detract Solenoids off
        solenoidDetract1.set(false);
        solenoidDetract2.set(false);
    }

    public void detractArm() { 
        // Turn Extender Solenoids off
        solenoidExtend1.set(false);
        solenoidExtend2.set(false);
        // Turn Detract Solenoids on
        solenoidDetract1.set(true);
        solenoidDetract2.set(true);
    }

    
    
    

}
