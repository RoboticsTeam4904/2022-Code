package org.usfirst.frc4904.robot.commands.shooter;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.motor.MotorIdle;

public class ShooterOff extends MotorIdle {

  /**
   * Set indexer motor speed to zero
   */
  public ShooterOff() {
      super(RobotMap.Component.shooterMotor);
  }
}
