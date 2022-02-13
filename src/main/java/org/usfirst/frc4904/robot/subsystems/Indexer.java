// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc4904.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.subsystems.motor.Motor;


public class Indexer extends SubsystemBase {

  public static final double DEFAULT_INDEXER_SPEED = .5; //TODO: Set value
  public final Motor indexerMotor1;
  public final Motor indexerMotor2;
  public Indexer(Motor indexerMotor1, Motor indexerMotor2) {
    this.indexerMotor1 = indexerMotor1;
		this.indexerMotor2 = indexerMotor2;
  }
}
