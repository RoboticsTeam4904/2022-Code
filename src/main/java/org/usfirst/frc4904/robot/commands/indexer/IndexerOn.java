package org.usfirst.frc4904.robot.commands.indexer;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.motor.MotorConstant;

public class IndexerOn extends MotorConstant {
    public final static double DEFAULT_INTAKE_MOTOR_SPEED = 0.5; //TODO: needs value
    public IndexerOn() {
        super("IndexerOn", RobotMap.Component.motor, DEFAULT_INTAKE_MOTOR_SPEED);
    }
}
