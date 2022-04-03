package org.usfirst.frc4904.robot.commands.shooter;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.motor.MotorBrake;

public class ShooterBrake extends MotorBrake {

  /**
   * Spin up the shooter to a speed
   * 
   * @param speed  The speed to spin the shooter up to
   */

  public ShooterBrake() {
    super("ShooterBrake", RobotMap.Component.shooterTalon);
  }
}