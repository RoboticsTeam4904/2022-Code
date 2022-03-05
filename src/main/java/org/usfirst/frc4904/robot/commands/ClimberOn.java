package org.usfirst.frc4904.robot.commands;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.motor.MotorConstant;

public class ClimberOn extends MotorConstant {
    public final static double DEFAULT_INTAKE_MOTOR_SPEED = 0.5; //TODO: needs value
    public ClimberOn() {
        super("ClimberOn", RobotMap.Component.climber, DEFAULT_INTAKE_MOTOR_SPEED);
    }
}