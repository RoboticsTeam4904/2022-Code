package org.usfirst.frc4904.robot.commands;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.motor.MotorConstant;

public class ClimberBOn extends MotorConstant {
    public final static double DEFAULT_CLIMBERA_MOTOR_B_SPEED = 0.5; //TODO: needs value
    public ClimberBOn() {
        super("ClimberBOn", RobotMap.Component.climberB, DEFAULT_CLIMBERA_MOTOR_B_SPEED);
    }
}