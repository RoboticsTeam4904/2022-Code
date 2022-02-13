package org.usfirst.frc4904.robot.commands.indexer;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.motor.MotorIdle;


public class IndexerOff extends MotorIdle {

  /**
   * Set indexer motor speed to zero
   */
  public IndexerOff() {
    super(RobotMap.Component.indexerMotor);
  }
}
