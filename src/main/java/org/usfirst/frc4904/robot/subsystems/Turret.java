// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc4904.robot.subsystems;

import org.usfirst.frc4904.standard.subsystems.motor.PositionSensorMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    public static final double ENCODER_TICKS = 1024;
	public static final double TICK_MULTIPLIER = 360.0 / ENCODER_TICKS; // TODO: CHANGE CONSTS, took this from 2019 elevator
    public final PositionSensorMotor turretMotor; // TODO: confirm type of motor

    /** Creates a new Turret. */
    public Turret(PositionSensorMotor turretMotor) {
        this.turretMotor = turretMotor;
    }
    /*
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
    */
}
