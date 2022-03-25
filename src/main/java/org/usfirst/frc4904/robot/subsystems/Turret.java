// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc4904.robot.subsystems;

import org.usfirst.frc4904.standard.custom.sensors.CANTalonEncoder;
import org.usfirst.frc4904.standard.subsystems.motor.PositionSensorMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    public static final double ENCODER_TICKS = 2048; // TODO: CHANGE CONST, took this from the 2019 elevator
	public static final double TICK_MULTIPLIER = (2 * Math.PI) / ENCODER_TICKS; 
    public static final double BIG_GEAR_RADIUS = 120; // Gear of the turret, TODO: CHANGE CONST
    public static final double SMALL_GEAR_RADIUS = 24; // Gear of the motor, TODO: CHANGE CONST
    public static final double GEAR_RATIO = BIG_GEAR_RADIUS / SMALL_GEAR_RADIUS;
    public final PositionSensorMotor turretMotor; // TODO: confirm type of motor
    public final CANTalonEncoder turretEncoder;
    public boolean swivvle = false; // TODO maybe tune

    /** Creates a new Turret. */
    public Turret(PositionSensorMotor turretMotor, CANTalonEncoder turretEncoder) {
        this.turretMotor = turretMotor;
        this.turretEncoder = turretEncoder;
    }

     /* Returns angle in radians. */
     public double getAngle() {
        return (turretEncoder.getDistance() * TICK_MULTIPLIER)/GEAR_RATIO;
    }
}
