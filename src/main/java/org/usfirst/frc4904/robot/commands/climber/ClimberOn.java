package org.usfirst.frc4904.robot.commands.climber;
import org.usfirst.frc4904.robot.subsystems.Climber;
import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.motor.MotorConstant;

public class ClimberOn extends MotorConstant {
    public ClimberOn() {
        super("ClimberOn", RobotMap.Component.climber, Climber.DEFAULT_CLIMBER_SPEED);
    }
}