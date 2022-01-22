package org.usfirst.frc4904.robot.commands.intake;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.motor.MotorConstant;

public class AxleIntakeOn extends MotorConstant {
    public AxleIntakeOn() {
        super("Axel Intake On", RobotMap.Component.intakeAxleMotor, 0.5); // todo: needs motor speed
    }
}