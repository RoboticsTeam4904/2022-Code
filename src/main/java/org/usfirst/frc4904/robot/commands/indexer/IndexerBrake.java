package org.usfirst.frc4904.robot.commands.indexer;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.motor.MotorBrake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;


public class IndexerBrake extends ParallelCommandGroup {

  /**
   * Brake indexer motors
   */
  public IndexerBrake() {
    this.addCommands(
      new MotorBrake("BrakeIndexerHolder", RobotMap.Component.indexerHolderTalon),
      new MotorBrake("BrakeIndexerBelt", RobotMap.Component.indexerBeltTalon)
    );
  }
}
