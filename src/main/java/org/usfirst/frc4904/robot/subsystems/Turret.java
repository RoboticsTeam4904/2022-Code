// Copyright (c) FIRST and other WPILib contributors.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc4904.robot.subsystems;

import org.usfirst.frc4904.standard.custom.sensors.CANTalonEncoder;
import org.usfirst.frc4904.standard.subsystems.motor.PositionSensorMotor;
import org.usfirst.frc4904.standard.LogKitten;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    public static final double TICKS_PER_REVM = 2048;
    public static final double RADIANS_PER_REV = 2 * Math.PI;
    public static final double BIG_GEAR_RADIUS = 120;
    public static final double SMALL_GEAR_RADIUS = 24;
    public static final double MOTOR_REV_PER_TURRET_REV = BIG_GEAR_RADIUS/SMALL_GEAR_RADIUS;

    public PositionSensorMotor motor;
    public CANTalonEncoder encoder;

    /** Creates a new Turret. */
    public Turret(PositionSensorMotor motor, CANTalonEncoder encoder) {
        super();
        this.motor = motor;
        this.encoder = encoder;
    }

    public static double convertTurretAngleToMotorPosition(double angle) {
		// Restrain position set to be within one turret rotation, reverse to other side if needed
		// final double pos =;                    // encoder ticks

		return normalizeAngle(angle, Math.PI)  // turret radians
		/ Turret.RADIANS_PER_REV                    // turret revs
		* Turret.MOTOR_REV_PER_TURRET_REV           // motor revs
		* Turret.TICKS_PER_REVM;
    }

    /// take a radian value and normalize to [-magnitude, magnitude]
    private static double normalizeAngle(double radians, double magnitude) {
        final var doubleMagnitude = 2 * magnitude;

        return (((radians + magnitude) % doubleMagnitude) + doubleMagnitude) % doubleMagnitude - magnitude;
    }

    /* Returns turret angle in radians, output in range [-pi, pi] */
    public double getAngle() {
        return this.encoder.getDistance() // ticks
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

    // take a radian value and normalize to [-pi, pi]
    // TODO remove, just a version of normalize that uses PI explicitly
    private static double turretModulo(double radians) {
        return (((radians + Math.PI) % (2*Math.PI)) + (2*Math.PI)) % (2*Math.PI) - Math.PI;
    }
}
