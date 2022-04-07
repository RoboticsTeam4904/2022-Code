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

<<<<<<< HEAD
  public ShooterSetSpeed() {
    super(RobotMap.Component.shooter.shooterMotor, Shooter.SHOOT_VELOCITY);

=======
  public ShooterSetSpeed(double velocity) {
    super(RobotMap.Component.shooter.shooterMotor, velocity);
>>>>>>> 0ce18e41c62be4c27bf4a2b1ada972ce18032880
  }
}