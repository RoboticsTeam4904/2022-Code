package org.usfirst.frc4904.robot.commands.intake;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.motor.MotorConstant;

/**
 * Rotates the axle to intake the ball
 */
public class AxleIntakeOn extends MotorConstant {
    public final static double DEFAULT_INTAKE_MOTOR_SPEED = 0.2; //TODO: needs value
    public AxleIntakeOn() {
        super("axleIntakeOn", RobotMap.Component.intakeAxleMotor, DEFAULT_INTAKE_MOTOR_SPEED);
    }
}