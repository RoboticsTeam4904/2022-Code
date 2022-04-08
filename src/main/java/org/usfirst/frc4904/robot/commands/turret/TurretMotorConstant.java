package org.usfirst.frc4904.robot.commands.turret;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.RobotMap.PID.Turret;
import org.usfirst.frc4904.standard.commands.motor.MotorConstant;

public class TurretMotorConstant extends MotorConstant{
    public TurretMotorConstant(double speed) {
        super(RobotMap.Component.turret.motor, speed);
        addRequirements(RobotMap.Component.turret);
    }
}
