// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.wpilibj.XboxController;
import frc.AutonomousCommands;

import java.net.PasswordAuthentication;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.time.*;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected = "default";
  private PIDCommand m_autonomousCommand;
  public static final DriveTrain robotDrive = new DriveTrain();
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private NetworkTable table;   
  private NetworkTableEntry tx, ty, ta, tv;
  

  //public static DriveTrain driveTrain = new DriveTrain();
 
    /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
         
    table = NetworkTableInstance.getDefault().getTable("limelight");
    table.getEntry("pipeline").setNumber(9);
    tx = table.getEntry("tx"); // Limelight's x-coordinate of the target
    ty = table.getEntry("ty"); // Limelight's y-coordinate of the target
    ta = table.getEntry("ta"); // Limelight's area of the target
    tv = table.getEntry("tv");
 
  // RobotMap.c.disable();
    RobotMap.c.enableDigital();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
   // AutonomousCommands.moveToID(9);
    //AutonomousCommands.ShootBall();
    switch(RobotMap.startingPosition){
      case 1:
       AutonomousCommands.moveToCube("left");
       AutonomousCommands.moveToID(1);
       AutonomousCommands.ShootBall();
       break;



       
    }
   
    //AutonomousCommands.moveToID(1);


  } 
 



 

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    boolean octaneDrive = false;
    double forwardPower;
    double turnMod;
  
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////UserVariables To Adjust/////////////////////////////////////////////////////////////////////////////////////
    double maxPower=1;
    double speedDecreaseOnChargePlate = 0.5;
    double turnStickDeadZone = 0.3;
    double driveStickDeadZone = 0.3;
   /* 
    if (RobotMap.driverController.getBButtonPressed()){
      if (RobotMap.compressorOn){
        RobotMap.c.disable();
        RobotMap.compressorOn=false;
      }else{
        RobotMap.c.enableDigital();
        RobotMap.compressorOn=true;
      }
    }
    */
    
    /////NonUserVariables///////////////////////////////////////////////////////////////////////////////////////////
   
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////SLOW MODE CODE SLOWS THE ROBOT for balancing on the charge plate/////////////////////////////////
    boolean slowMode = false;
    if(RobotMap.driverController.getLeftTriggerAxis() >= 0.8) {
      slowMode = true; 
    }
    
    if (slowMode){
      if (RobotMap.slowModeMultiplier==1)
{
  RobotMap.slowModeMultiplier=0;
}    else{
  RobotMap.slowModeMultiplier=1;
}
    }

    
  
   System.out.println(RobotMap.slowModeMultiplier);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////Driving/////////////////////////////////////////////////////////////////////////////////////////////
    turnMod = RobotMap.driverController.getRightX();
    forwardPower = RobotMap.driverController.getLeftY();

    if(Math.abs(turnMod) <= turnStickDeadZone) turnMod = 0;      //deadzone for turning stick
    if(Math.abs(forwardPower) <= driveStickDeadZone) forwardPower = 0;    //deadzone for driving stick

    turnMod *= 0.5;
    
    forwardPower *= maxPower - (RobotMap.slowModeMultiplier*speedDecreaseOnChargePlate);

    RobotMap.driveSystem.arcadeDrive(forwardPower,turnMod);


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////Pneumatics///////////////////////////////////////////////////////////////////////////////////////////////
   if (RobotMap.operatorController.getPOV()>330 ||RobotMap.operatorController.getPOV()<20 &&RobotMap.operatorController.getPOV()!=-1){
    Pneumatics.halfup();
       // Pneumatics.raise();
   }  
   if (RobotMap.operatorController.getPOV()<200&&RobotMap.operatorController.getPOV()>160){
    
    Pneumatics.lower();
  }   
  if (85<RobotMap.operatorController.getPOV()&&95>RobotMap.operatorController.getPOV()){
    Pneumatics.raise();
  }
  if (RobotMap.operatorController.getPOV()<310&&RobotMap.operatorController.getPOV()>230){
    Pneumatics.halflower();
  }


//operator
  if (RobotMap.operatorController.getLeftBumperPressed()){
    Pneumatics.openClaw();
  }
  if (RobotMap.operatorController.getBButtonPressed()){
  ///  Pneumatics.stopClaw();
    //Pneumatics.getPylon();
    Pneumatics.extendClaw();
}
//operator
  if (RobotMap.operatorController.getXButtonPressed()){
    Pneumatics.getCube();
    
      
  }
  ///operator
  if (RobotMap.operatorController.getYButtonPressed()){
    Pneumatics.getPylon();
    
  }

  //operator
  if (RobotMap.operatorController.getAButtonPressed()){
    Pneumatics.detractClaw();
  }if (RobotMap.operatorController.getRightY()<0){
    Pneumatics.extendClaw();
  }if (RobotMap.operatorController.getRightY()>0){
    Pneumatics.detractClaw();
  }
  ///operator
  if (RobotMap.operatorController.getRightBumperPressed()){
    Pneumatics.extendClaw();
    Pneumatics.openClaw();
  }



   //driveTrain.arcadeDr0ive(RobotMap.driverController);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
