package org.usfirst.frc4904.robot.commands.shooter;

import org.usfirst.frc4904.robot.Robot;
import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.standard.commands.motor.MotorVelocitySet;
import org.usfirst.frc4904.standard.subsystems.motor.VelocitySensorMotor;


public class SetDefaultShooterVelocity extends MotorVelocitySet {
    public SetDefaultShooterVelocity(VelocitySensorMotor motor) {
        super(motor, 0.2);
    }
}
