// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc4904.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.usfirst.frc4904.standard.subsystems.motor.Motor;


public class Indexer extends SubsystemBase {
  public static final double DEFAULT_INDEXER_SPEED = 0.5; //TODO: Set value
  public static final double DEFAULT_INTAKE_SPEED = 0.5;
  public final Motor holderMotor;
  public final Motor beltMotor;
  public Indexer(Motor holder, Motor belt) {
    this.holderMotor = holder;
	  this.beltMotor = belt;
  }
}
