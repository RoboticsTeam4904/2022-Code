package org.usfirst.frc4904.robot.commands;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.motor.MotorConstant;

public class ClimberAOn extends MotorConstant {
    public final static double DEFAULT_CLIMBER_MOTOR_A_SPEED = 0.5; //TODO: needs value
    public ClimberAOn() {
        super("ClimberAOn", RobotMap.Component.climberA, DEFAULT_CLIMBER_MOTOR_A_SPEED);
    }
}