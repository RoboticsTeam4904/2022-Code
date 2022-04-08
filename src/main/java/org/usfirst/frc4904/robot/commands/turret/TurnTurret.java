package org.usfirst.frc4904.robot.commands.turret;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.subsystems.Turret;
import org.usfirst.frc4904.standard.custom.sensors.InvalidSensorException;
import org.usfirst.frc4904.standard.subsystems.motor.PositionSensorMotor;
import org.usfirst.frc4904.standard.LogKitten;
import org.usfirst.frc4904.standard.commands.motor.MotorBrake;
import org.usfirst.frc4904.standard.commands.motor.MotorPositionConstant;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnTurret extends MotorPositionConstant {
	private final Turret turret;

	public TurnTurret(double angle, Turret turret) {
		super(turret.getMotor(), Turret.convertTurretAngleToMotorPosition(angle));

		this.turret = turret;

		addRequirements(RobotMap.Component.turret); // TODO: cringe (zach)
	}

	@Override
	public void initialize() {
		super.initialize();
		RobotMap.Component.turretMotor.setCoastMode();
	}

	@Override
	public boolean isFinished() {
		if (turret.isLimitSwitchClosed()) {
			return true;
		}

		return super.isFinished();
	}

	public void end(boolean interrupted) {
		super.end(interrupted);
		RobotMap.Component.turretMotor.setBrakeMode();
	}
}
