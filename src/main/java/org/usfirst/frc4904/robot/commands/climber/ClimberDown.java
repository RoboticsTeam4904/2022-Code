package org.usfirst.frc4904.robot.commands.climber;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.subsystems.Climber;
import org.usfirst.frc4904.standard.commands.motor.MotorConstant;

public class ClimberDown extends MotorConstant {
    public ClimberDown() {
        super("ClimberOn", RobotMap.Component.climber.climberMotor, -0.2);
    }
}