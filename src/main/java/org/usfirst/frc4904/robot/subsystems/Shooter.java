// Copyright (c) FIRST and other WPILib contributors.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc4904.robot.subsystems;

import org.usfirst.frc4904.robot.Robot;
import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.LogKitten;
import org.usfirst.frc4904.standard.custom.motioncontrollers.CANTalonFX;
import org.usfirst.frc4904.standard.custom.sensors.CANTalonEncoder;
import org.usfirst.frc4904.standard.subsystems.motor.Motor;
import org.usfirst.frc4904.standard.subsystems.motor.PositionSensorMotor;
import org.usfirst.frc4904.standard.subsystems.motor.VelocitySensorMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    public static final double SHOOT_VELOCITY = -1; //TODO: Set value and ball reject velocity (unused)
    public VelocitySensorMotor shooterMotor;
    public CANTalonEncoder shooterEncoder;

    public boolean armed = false;

    private double last = 0;
    private int counter = 0;

    public Shooter(VelocitySensorMotor shooterMotor, CANTalonEncoder shooterEncoder) {
        super();
        this.shooterMotor = shooterMotor;
        this.shooterEncoder = shooterEncoder;
    }

    @Override
    public void periodic() { // we cannot set a custom frequency for our subsystem periodic
        if (counter == 5) {
            double currentVelocity = RobotMap.Component.shooterTalon.getSelectedSensorVelocity();
            double outputPercent = RobotMap.Component.shooterTalon.getMotorOutputPercent();
            System.out.println(String.format("velocity %f and outpute percent %f", currentVelocity, outputPercent));

            if (Math.abs(currentVelocity-last) < 1) {
                armed = true;
            } else {
                armed = false;
            }
            counter = 0;
        }
        counter += 1;
    }
}
