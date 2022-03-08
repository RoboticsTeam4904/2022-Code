package org.usfirst.frc4904.robot.commands.intake;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.motor.MotorConstant;

/**
 * Rotates the motor to intake the ball
 */
public class SecondaryIntakeOn extends MotorConstant {
    public final static double DEFAULT_INTAKE_MOTOR_SPEED = 0.5; //TODO: needs value
    public SecondaryIntakeOn() {
        super("SecondaryIntakeOn", RobotMap.Component.intakeSecondaryMotor, DEFAULT_INTAKE_MOTOR_SPEED);
    }
}