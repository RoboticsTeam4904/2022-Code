// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc4904.robot.subsystems;

import org.usfirst.frc4904.standard.custom.sensors.CANTalonEncoder;
import org.usfirst.frc4904.standard.subsystems.motor.PositionSensorMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    public static final double TICKS_PER_REVM = 2048;
    public static final double RADIANS_PER_REV = 2 * Math.PI;
    public static final double BIG_GEAR_RADIUS = 120;
    public static final double SMALL_GEAR_RADIUS = 24;
    public static final double MOTOR_REV_PER_TURRET_REV = BIG_GEAR_RADIUS / SMALL_GEAR_RADIUS;

    private final PositionSensorMotor motor;
    private final CANTalonEncoder encoder;

    /** Creates a new Turret. */
    public Turret(PositionSensorMotor motor, CANTalonEncoder encoder) {
        this.motor = motor;
        this.encoder = encoder;
    }

    public static double convertTurretAngleToMotorPosition(double angle) {
        // Restrain position set to be within one turret rotation, reverse to other side
        // if needed
        return normalizeAngle(angle, Math.PI) // turret radians
                / RADIANS_PER_REV // turret revs
                * MOTOR_REV_PER_TURRET_REV // motor revs
                * TICKS_PER_REVM; // encoder ticks
    }

    /// take a radian value and normalize to [-magnitude, magnitude]
    private static double normalizeAngle(double radians, double magnitude) {
        final var doubleMagnitude = 2 * magnitude;

        return (((radians + magnitude) % doubleMagnitude) + doubleMagnitude) % doubleMagnitude - magnitude;
    }

    /* Returns turret angle in radians, output in range [-pi, pi] */
    public double getAngle() {
        return encoder.getDistance() // ticks
                / TICKS_PER_REVM // motor revolutions
                / MOTOR_REV_PER_TURRET_REV // turret revolutions
                * RADIANS_PER_REV; // turret radians
    }

    public boolean isLimitSwitchClosed() {
        return encoder.isFwdLimitSwitchClosed() || encoder.isRevLimitSwitchClosed();
    }

    public PositionSensorMotor getMotor() {
        return motor;
    }
}
