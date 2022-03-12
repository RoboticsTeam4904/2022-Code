package org.usfirst.frc4904.robot.commands;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.motor.MotorIdle;


public class ClimberAOff extends MotorIdle {

  /**
   * Set indexer motor speed to zero
   */
  public ClimberAOff() {
    super(RobotMap.Component.climberA);
  }
}