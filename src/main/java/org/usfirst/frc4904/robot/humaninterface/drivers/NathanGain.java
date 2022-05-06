package org.usfirst.frc4904.robot.humaninterface.drivers;

import org.usfirst.frc4904.robot.Robot;
import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.commands.climber.ClimberBrake;
import org.usfirst.frc4904.robot.commands.climber.ClimberDown;
import org.usfirst.frc4904.robot.commands.climber.ClimberOff;
import org.usfirst.frc4904.robot.commands.climber.ClimberUp;
import org.usfirst.frc4904.robot.commands.indexerIntakeTurret.IndexerIntakeOff;
import org.usfirst.frc4904.robot.commands.indexerIntakeTurret.StoreBall;
import org.usfirst.frc4904.robot.commands.intake.ExtendIntake;
import org.usfirst.frc4904.robot.commands.intake.RetractIntake;
import org.usfirst.frc4904.robot.commands.shooter.ShooterSetSpeed;
import org.usfirst.frc4904.robot.commands.turret.TurnTurret;
import org.usfirst.frc4904.standard.LogKitten;
import org.usfirst.frc4904.standard.commands.RunFor;
import org.usfirst.frc4904.standard.commands.chassis.ChassisShift;
import org.usfirst.frc4904.standard.commands.motor.MotorBrake;
import org.usfirst.frc4904.standard.commands.motor.MotorConstant;
import org.usfirst.frc4904.standard.commands.solenoid.SolenoidExtend;
import org.usfirst.frc4904.standard.commands.solenoid.SolenoidRetract;
import org.usfirst.frc4904.standard.humaninput.Driver;
import org.usfirst.frc4904.standard.subsystems.chassis.SolenoidShifters;
import org.usfirst.frc4904.robot.commands.indexerIntakeTurret.BallReject;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.usfirst.frc4904.robot.RobotMap;

public class NathanGain extends Driver {
	public static final double SPEED_GAIN = 1;
	public static final double SPEED_EXP = 2;
	public static final double TURN_GAIN = 0.6;
	public static final double TURN_EXP = 1;
	public static final double Y_SPEED_SCALE = 1;
	public static final double TURN_SPEED_SCALE = 1;

	public NathanGain() {
		super("NathanGain");
	}

	protected double scaleGain(double input, double gain, double exp) {
		return Math.pow(Math.abs(input), exp) * gain * Math.signum(input);
	}

	@Override
	public void bindCommands() {
		RobotMap.HumanInput.Driver.xbox.y.whenPressed(new SolenoidRetract(RobotMap.Component.chassisSolenoid));
		RobotMap.HumanInput.Driver.xbox.y.whenReleased(new SolenoidExtend(RobotMap.Component.chassisSolenoid));
		//RobotMap.HumanInput.Driver.xbox.y.whenReleased(new ChassisShift(RobotMap.Component.chassis.getShifter(), SolenoidShifters.SolenoidState.RETRACT));
		RobotMap.HumanInput.Driver.xbox.a.whenPressed(new RunFor(new StoreBall(), 3.0));
		RobotMap.HumanInput.Driver.xbox.rb.whenPressed(new ExtendIntake());
		RobotMap.HumanInput.Driver.xbox.lb.whenPressed(new RetractIntake());
		RobotMap.HumanInput.Driver.xbox.x.whenPressed(new RunFor(new BallReject(), 3.0));
		// RobotMap.HumanInput.Driver.xbox.x.whenPressed(new MotorBrake(RobotMap.Component.shooterTalon));

		// RobotMap.HumanInput.Driver.xbox.dPad.up.whenPressed(new TurnTurret(Math.PI / 2, RobotMap.Component.turret));
		// RobotMap.HumanInput.Driver.xbox.dPad.left.whenPressed(new TurnTurret(Math.PI / 4, RobotMap.Component.turret));
		// RobotMap.HumanInput.Driver.xbox.dPad.right.whenPressed(new MotorConstant(RobotMap.Component.turret.motor, 0.06));
		// RobotMap.HumanInput.Driver.xbox.dPad.down.whenPressed(new TurnTurret(0, RobotMap.Component.turret));
	}

	@Override
	public double getX() {
		return 0;
	}

	@Override
	public double getY() {
		double rawSpeed = RobotMap.HumanInput.Driver.xbox.rt.getX() - RobotMap.HumanInput.Driver.xbox.lt.getX();
		double speed = scaleGain(rawSpeed, NathanGain.SPEED_GAIN, NathanGain.SPEED_EXP) * NathanGain.Y_SPEED_SCALE;
		double precisionDrive = scaleGain(RobotMap.HumanInput.Driver.xbox.rightStick.getY(), 0.08, 1.2);
		// LogKitten.wtf("joystick y " + RobotMap.HumanInput.Operator.joystick.getAxis(1));
		double operatorDrive = scaleGain(-RobotMap.HumanInput.Operator.joystick.getAxis(1), 0.1, 1.2);
		return speed + precisionDrive + operatorDrive;
	}

	@Override
	public double getTurnSpeed() {
		double rawTurnSpeed = RobotMap.HumanInput.Driver.xbox.leftStick.getX();
		double precisionTurnSpeed = scaleGain(RobotMap.HumanInput.Driver.xbox.rightStick.getX(), 0.08, 1.2);
		double operatorControlTurnSpeed = scaleGain(RobotMap.HumanInput.Operator.joystick.getAxis(0), 0.2, 1.5);
		double turnSpeed = scaleGain(rawTurnSpeed, NathanGain.TURN_GAIN, NathanGain.TURN_EXP)
				* NathanGain.TURN_SPEED_SCALE;
		return turnSpeed + precisionTurnSpeed + operatorControlTurnSpeed;
		// return turnSpeed;
	}
}