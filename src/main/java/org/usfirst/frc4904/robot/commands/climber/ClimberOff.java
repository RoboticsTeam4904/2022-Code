package org.usfirst.frc4904.robot.commands.climber;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.LogKitten;
import org.usfirst.frc4904.standard.commands.motor.MotorIdle;
public class ClimberOff extends MotorIdle {
    public ClimberOff() {
        super("ClimberOff", RobotMap.Component.climber.climberMotor);
    }
}