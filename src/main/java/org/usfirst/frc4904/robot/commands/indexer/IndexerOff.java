package org.usfirst.frc4904.robot.commands.indexer;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.motor.MotorIdle;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;


public class IndexerOff extends ParallelCommandGroup {

  /**
   * Set indexer motor speed to zero
   */
  public IndexerOff() {
    this.addCommands(
      new MotorIdle(RobotMap.Component.indexer.holderMotor),
      new MotorIdle(RobotMap.Component.indexer.beltMotor)
    );
  }
}
