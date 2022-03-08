package org.usfirst.frc4904.robot.commands;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.subsystems.Flywheel;

public class FlywheelOff extends FlywheelSetSpeed {

  /**
   * Set flywheel speed to zero
   * 
   * @param flywheel The flywheel to manipulate
   */
  public FlywheelOff(Flywheel flywheel) {
    super(flywheel, 0.0);
  }

  public FlywheelOff() {
    super(RobotMap.Component.shooter, 0.0);
  }
}