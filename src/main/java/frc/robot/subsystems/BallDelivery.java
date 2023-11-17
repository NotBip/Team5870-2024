/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.commands.Commands;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import frc.robot.util.DoubleSolenoidToggler;

/**
 * The launcher subsystem on the robot. Can intake and shoot cargo.
 */
public class BallDelivery extends SubsystemBase {

  private Victor leftMotor = new Victor(RobotMap.LEFT_MOTOR);
  private Victor rightMotor = new Victor(RobotMap.RIGHT_MOTOR);

  private DoubleSolenoidToggler pistons = RobotMap.pistons;

  public BallDelivery() {
    raise();
    setDefaultCommand(Commands.launch);
  }

  /** Sets the power of the launcher intake/outtake wheels to a certain value (between -1 and 1). */
  public void power(double power) {
    power = Math.min(1.0, power);
    power = Math.max(-1.0, power);

    if(Math.abs(power) < 0.1) {
      stop(); 
    } else {
      leftMotor.set(power);
      rightMotor.set(-power);
    }
  }

  /** Stop both intake/outtake wheels. */
  public void stop() {
    leftMotor.set(0);
    rightMotor.set(0);
  }

  public void togglePistons() {
    pistons.toggle();
  }

  public void raise() {
    pistons.retract();
  }

  public void lower() {
    pistons.extend();
  }

  public boolean isRaised() {
    return !pistons.extended;
  }

  public double getSpeed() {
    return leftMotor.get();
  }
}
