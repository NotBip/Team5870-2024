package frc.robot;

import edu.wpi.first.wpilibj.Timer;

public class Pneumatics {
    public static void raise(){
        RobotMap.solenoidUpDown1.set(false);
        RobotMap.solenoidUpDown2.set(true);

    }
    public static void lower(){
        RobotMap.solenoidUpDown1.set(true);
        RobotMap.solenoidUpDown2.set(false);
    }
    
    public static void extendClaw(){
        RobotMap.solenoidInOut1.set(false);
        RobotMap.solenoidInOut2.set(true);

    }
    public static void detractClaw(){
        RobotMap.solenoidInOut1.set(true);
        RobotMap.solenoidInOut2.set(false);

    }

    public static void stopClaw(){
        RobotMap.solenoidUpDown1.set(false);
        RobotMap.solenoidUpDown2.set(false);
    }
    
    public static void halfup(){
        RobotMap.solenoidUpDown1.set(false);
        RobotMap.solenoidUpDown2.set(true);
      Timer.delay(1.3); 
      RobotMap.solenoidUpDown2.set(false);
    }
    
    public static void halflower(){
        RobotMap.solenoidUpDown1.set(true);
        RobotMap.solenoidUpDown2.set(false);
        Timer.delay(0.4); 
        RobotMap.solenoidUpDown1.set(false);

    }
    public static void openClaw(){
        RobotMap.solenoidOpenClose1.set(true);
        RobotMap.solenoidOpenClose2.set(true);

    }
    public static void getPylon(){
        RobotMap.solenoidOpenClose1.set(false);
        RobotMap.solenoidOpenClose2.set(false);

    }

    public static void getCube(){
        RobotMap.solenoidOpenClose1.set(false);
        RobotMap.solenoidOpenClose2.set(true);
        
    }

}
