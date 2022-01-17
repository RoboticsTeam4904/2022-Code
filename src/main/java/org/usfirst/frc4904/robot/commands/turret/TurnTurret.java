package org.usfirst.frc4904.robot.commands.turret;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.custom.sensors.InvalidSensorException;
import org.usfirst.frc4904.standard.subsystems.motor.PositionSensorMotor;
import org.usfirst.frc4904.standard.LogKitten;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnTurret extends CommandBase {
	protected PositionSensorMotor turretMotor;
	protected double turretPosition;

	public TurnTurret(String name, double turretPosition) {
		super();
		this.setName(name);
		this.turretMotor = RobotMap.Component.turret.turretMotor;
		this.turretPosition = turretPosition;
		addRequirements(RobotMap.Component.turret);
	}
	
	public TurnTurret(double turretPosition) {
		super();
		this.setName("TurnTurret");
		this.turretMotor = RobotMap.Component.turret.turretMotor;
		this.turretPosition = turretPosition;
		addRequirements(RobotMap.Component.turret);
	}

	@Override
	public void initialize() {
		try {
			this.turretMotor.reset();
			this.turretMotor.enableMotionController();
			this.turretMotor.setPositionSafely(turretPosition); // TODO: gear ratios 
		} catch (InvalidSensorException e) {
			LogKitten.e("InvalidSensorException in TurnTurret.initialize()");
			cancel();
		}
	}

	@Override
	public void execute() {
		Exception potentialSensorException = this.turretMotor.checkSensorException();
		if (potentialSensorException != null) {
			LogKitten.e("SensorException in TurnTurret.execute()");
			cancel();
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
