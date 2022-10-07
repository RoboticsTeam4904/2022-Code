package org.usfirst.frc4904.robot.commands.intake;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.motor.MotorConstant;

/**
 * Rotates the axle to intake the ball
 */
public class AxleIntakeOn extends MotorConstant {
    public final static double DEFAULT_INTAKE_MOTOR_SPEED = 0.5; //TODO: needs value
    public AxleIntakeOn() {
        super("axleIntakeOn", RobotMap.Component.intakeAxleMotor, DEFAULT_INTAKE_MOTOR_SPEED);
    }

    public AxleIntakeOn(boolean reverse) { // TODO ok this is immensley cringe but I don't want to deal with java right now
        super("axleIntakeOn", RobotMap.Component.intakeAxleMotor, -DEFAULT_INTAKE_MOTOR_SPEED);
    }
}