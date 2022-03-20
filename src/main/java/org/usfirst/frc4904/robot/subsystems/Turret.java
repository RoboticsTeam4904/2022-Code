// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc4904.robot.subsystems;

import org.usfirst.frc4904.standard.subsystems.motor.PositionSensorMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    public static final double ENCODER_TICKS = 1024; // TODO: CHANGE CONST, took this from the 2019 elevator
	public static final double TICK_MULTIPLIER = 360.0 / ENCODER_TICKS; 
    public static final double BIG_GEAR_RADIUS = 120; // Gear of the turret, TODO: CHANGE CONST
    public static final double SMALL_GEAR_RADIUS = 24; // Gear of the motor, TODO: CHANGE CONST
    public static final double GEAR_RATIO = BIG_GEAR_RADIUS / SMALL_GEAR_RADIUS;
    public final PositionSensorMotor turretMotor; // TODO: confirm type of motor

    /** Creates a new Turret. */
    public Turret(PositionSensorMotor turretMotor) {
        this.turretMotor = turretMotor;
    }
}
