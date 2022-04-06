// Copyright (c) FIRST and other WPILib contributors.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc4904.robot.subsystems;

import org.usfirst.frc4904.standard.custom.motioncontrollers.CANTalonFX;
import org.usfirst.frc4904.standard.custom.sensors.CANTalonEncoder;
import org.usfirst.frc4904.standard.subsystems.motor.Motor;
import org.usfirst.frc4904.standard.subsystems.motor.PositionSensorMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    public static final double DEFAULT_SHOOTER_SPEED = -1; //TODO: Set value
    public Motor flywheel;
    public CANTalonFX encoder;

    public Shooter(Motor flywheel, CANTalonFX shooterTalon) {
        this.flywheel = flywheel;
        this.encoder = shooterTalon;
    }


}
