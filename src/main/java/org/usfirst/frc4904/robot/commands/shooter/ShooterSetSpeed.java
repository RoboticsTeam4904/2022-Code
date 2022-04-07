package org.usfirst.frc4904.robot.commands.shooter;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.subsystems.Shooter;
import org.usfirst.frc4904.standard.commands.motor.MotorConstant;
import org.usfirst.frc4904.standard.commands.motor.MotorVelocitySet;

public class ShooterSetSpeed extends MotorVelocitySet {

  /**
   * Spin up the shooter to a speed
   * 
   * @param speed  The speed to spin the shooter up to
   */

  public ShooterSetSpeed() {
    super(RobotMap.Component.shooter.shooterMotor, Shooter.DEFAULT_VELOCITY);

  }
}