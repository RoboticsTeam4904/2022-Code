package org.usfirst.frc4904.robot.commands.climber;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.LogKitten;
import org.usfirst.frc4904.standard.commands.motor.MotorBrake;
import org.usfirst.frc4904.standard.commands.motor.MotorIdle;
public class ClimberBrake extends MotorBrake {
    public ClimberBrake() {
        super("ClimberBrake", RobotMap.Component.climber.climberTalon);
    }
}
