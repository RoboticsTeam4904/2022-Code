package org.usfirst.frc4904.robot.commands;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.subsystems.Flywheel;
import org.usfirst.frc4904.standard.commands.motor.MotorVelocitySet;

public class FlywheelSetSpeed extends MotorVelocitySet {

  /**
   * Spin up the flywheel to a speed
   * 
   * @param flywheel The flywheel to manipulate
   * @param speed    The speed to spin the flywheel up to
   */
  public FlywheelSetSpeed(Flywheel flywheel, double speed) {
    super("FlywheelMaintainSpeed", flywheel, speed);
  }

  public FlywheelSetSpeed(double speed) {
    this(RobotMap.Component.shooter, speed);
  }
}