package org.usfirst.frc4904.robot.commands.climber;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.subsystems.Climber;
import org.usfirst.frc4904.standard.commands.motor.MotorConstant;

public class ClimberUp extends MotorConstant {
    public ClimberUp() {
        super("ClimberOn", RobotMap.Component.climber.climberMotor, -0.5);
    }
}