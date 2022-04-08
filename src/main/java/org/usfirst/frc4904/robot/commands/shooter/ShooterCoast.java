package org.usfirst.frc4904.robot.commands.shooter;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.motor.MotorBrake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterCoast extends CommandBase {

  /**
   * Spin up the shooter to a speed
   * 
   * @param speed  The speed to spin the shooter up to
   */

  @Override
  public void initialize() {
    super.initialize();
    RobotMap.Component.shooterTalon.setCoastMode();
  }
}