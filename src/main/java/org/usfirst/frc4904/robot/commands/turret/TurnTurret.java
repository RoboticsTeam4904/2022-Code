package org.usfirst.frc4904.robot.commands.turret;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.subsystems.Turret;
import org.usfirst.frc4904.standard.custom.sensors.InvalidSensorException;
import org.usfirst.frc4904.standard.subsystems.motor.PositionSensorMotor;
import org.usfirst.frc4904.standard.LogKitten;
import org.usfirst.frc4904.standard.commands.motor.MotorPositionConstant;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnTurret extends MotorPositionConstant {
	public TurnTurret(double turretRadians) {
		// Restrain position set to be within one turret rotation, reverse to other side if needed
		super(RobotMap.Component.turret.turretMotor, ((turretRadians % (2 * Math.PI) - Math.PI) * (1 / Turret.TICK_MULTIPLIER) * Turret.GEAR_RATIO));
	}
}
