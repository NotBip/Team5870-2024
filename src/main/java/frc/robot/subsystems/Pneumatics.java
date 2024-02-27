package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Pneumatics {
    
    Compressor c; 
    boolean CompressorOn = false; 

    public Pneumatics() { 
        c = new Compressor(0, PneumaticsModuleType.CTREPCM); 
    }

    

}
