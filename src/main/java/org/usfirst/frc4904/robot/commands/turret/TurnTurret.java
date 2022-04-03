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
		// final double pos =;                    // encoder ticks

		super(RobotMap.Component.turret.turretMotor,  turretModulo(turretRadians)  // turret radians
		/ Turret.RADIANS_PER_REV                    // turret revs
		* Turret.MOTOR_REV_PER_TURRET_REV           // motor revs
		* Turret.TICKS_PER_REVM);
		addRequirements(RobotMap.Component.turret);
	}

	@Override
	public boolean isFinished() {
		if (RobotMap.Component.turret.turretEncoder.getFwdLimitSwitchClosed() == 1 || RobotMap.Component.turret.turretEncoder.getRevLimitSwitchClosed() == 1) {
			return true;
		}
		if(super.isFinished()) LogKitten.wtf("turret command end");
		return super.isFinished();
	}
	
	/// take a radian value and normalize to [-pi, pi]
	private static double turretModulo(double radians) {
		return (((radians + Math.PI) % (2*Math.PI)) + (2*Math.PI)) % (2*Math.PI) - Math.PI;
	}
}
