package frc;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Pneumatics;
public class AutonomousCommands {
   public void leaveCommunity(){

   }

  
    public static void moveToCube(String turn){
      NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
      table.getEntry("pipeline").setNumber(9);
      NetworkTableEntry tx = table.getEntry("tx"); // Limelight's x-coordinate of the target
      NetworkTableEntry ty = table.getEntry("ty"); // Limelight's y-coordinate of the target
      NetworkTableEntry ta = table.getEntry("ta"); // Limelight's area of the target
      NetworkTableEntry tv = table.getEntry("tv");
    
      while (true){
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        double x = tx.getDouble(0.0);
        double prevarea = 0; 
        
        if(area == 0) {
            if (turn.equals("left")){
                Robot.robotDrive.arcadeDriveSet(0, -0.2);
            }else{
                Robot.robotDrive.arcadeDriveSet(0,0.2);
            }
          
        }
        else {
          if (area==90){
            break;
          }
          if(prevarea < area && area >= 1.2) {
            Robot.robotDrive.arcadeDriveSet(0, 0); 
            Pneumatics.openClaw();
            Timer.delay(1); 
            Pneumatics.getCube();
            break;
          }
          else {
            Robot.robotDrive.arcadeDriveSet(RobotMap.forwardP*(43-area),RobotMap.p*(x-3.0));
          }
        }
        prevarea = area; 
      } 
    }

    public void moveToCone(String turn){
      NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
      table.getEntry("pipeline").setNumber(0);
      NetworkTableEntry tx = table.getEntry("tx"); // Limelight's x-coordinate of the target
      NetworkTableEntry ty = table.getEntry("ty"); // Limelight's y-coordinate of the target
      NetworkTableEntry ta = table.getEntry("ta"); // Limelight's area of the target
      NetworkTableEntry tv = table.getEntry("tv");
    
      while (true){
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        double x = tx.getDouble(0.0);
        double prevarea = 0; 
        
        if(area == 0) {
            if (turn.equals("left")){
                Robot.robotDrive.arcadeDriveSet(0, -0.2);
            }else{
                Robot.robotDrive.arcadeDriveSet(0,0.2);
            }
          
        }
        else {
          if (area==90){
            break;
          }
          if(prevarea < area && area >= 1.2) {
            Robot.robotDrive.arcadeDriveSet(0, 0); 
            Pneumatics.openClaw();
            break; 
          }
          else {
            Robot.robotDrive.arcadeDriveSet(RobotMap.forwardP*(43-area),RobotMap.p*(x-3.0));
          }
        }
        prevarea = area; 
    }
    Pneumatics.getPylon(); 
   }


   public static void ShootBall() {
      Pneumatics.halfup();
      Pneumatics.extendClaw();

      double movetime = 0.6; // just keep testing this, only this needs to be adjusted since it's the time for movement

      Timer.delay(0.6); // gives the pneumatics time to move
      Timer mtimer = new Timer();
      mtimer.start();
      while (mtimer.get() < movetime) {
        Robot.robotDrive.arcadeDriveSet(0.6, 0); 
      // maybe adjust the speed but this should be fast enough to score but slow enough to not break the robot
      }
      Pneumatics.openClaw();

    }

    public static void moveToID(int id){
      NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
      table.getEntry("pipeline").setNumber(id);
      NetworkTableEntry tx = table.getEntry("tx"); // Limelight's x-coordinate of the target
      NetworkTableEntry ty = table.getEntry("ty"); // Limelight's y-coordinate of the target
      NetworkTableEntry ta = table.getEntry("ta"); // Limelight's area of the target
      NetworkTableEntry tv = table.getEntry("tv");
    
      while (true){
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        double x = tx.getDouble(0.0);
        double prevarea = 0; 
        
        if(area == 0) {
          Robot.robotDrive.arcadeDriveSet(0,0.2);
        }
        else {
          if (area==90){
            break;
          }
          if(prevarea < area && area >= 25) {
            Robot.robotDrive.arcadeDriveSet(0, 0); 
            break; 
          }
          else {
            Robot.robotDrive.arcadeDriveSet(RobotMap.forwardP*(43-area),RobotMap.p*(x-3.0));
          }
        }
        prevarea = area; 
      }
     }
}
