package org.usfirst.frc4904.robot.commands.shooter;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.subsystems.Shooter;
import org.usfirst.frc4904.standard.LogKitten;
import org.usfirst.frc4904.standard.commands.motor.MotorConstant;
import org.usfirst.frc4904.standard.commands.motor.MotorVelocitySet;

public class ShooterConstant extends MotorConstant {

  /**
   * Spin up the shooter to a speed
   * 
   * @param speed  The speed to spin the shooter up to
   */

  public ShooterConstant(double speed) {
    super(RobotMap.Component.shooterMotor, speed);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);

    LogKitten.wtf("wtf");
  }
}