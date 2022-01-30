package org.usfirst.frc4904.robot.commands;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.motor.MotorIdle;
import org.usfirst.frc4904.standard.subsystems.motor.Motor;

public class IndexerOff extends MotorIdle {

  /**
   * Set motor speed to zero
   * 
   * @param motor The motor to manipulate
   */
  public IndexerOff(Motor motor) {
    super(motor, 0.0);
  }

  public IndexerOff() {
    super(RobotMap.Component.motor, 0.0);
  }
}
