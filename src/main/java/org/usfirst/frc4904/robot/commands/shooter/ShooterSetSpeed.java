package org.usfirst.frc4904.robot.commands.shooter;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.subsystems.Shooter;
import org.usfirst.frc4904.standard.commands.motor.MotorConstant;

public class ShooterSetSpeed extends MotorConstant {

  /**
   * Spin up the shooter to a speed
   * 
   * @param speed  The speed to spin the shooter up to
   */

  public ShooterSetSpeed(double speed) {
    super(RobotMap.Component.shooter.flywheel, Shooter.DEFAULT_SHOOTER_SPEED);

  }
}