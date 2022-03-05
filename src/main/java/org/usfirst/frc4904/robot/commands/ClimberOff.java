package org.usfirst.frc4904.robot.commands;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.motor.MotorIdle;


public class ClimberOff extends MotorIdle {

  /**
   * Set indexer motor speed to zero
   */
  public ClimberOff() {
    super(RobotMap.Component.climber);
  }
}