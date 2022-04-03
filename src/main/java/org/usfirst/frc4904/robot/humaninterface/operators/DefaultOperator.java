package org.usfirst.frc4904.robot.humaninterface.operators;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.commands.climber.ClimberDown;
import org.usfirst.frc4904.robot.commands.climber.ClimberOff;
import org.usfirst.frc4904.robot.commands.climber.ClimberUp;
import org.usfirst.frc4904.robot.commands.indexer.IndexerOff;
import org.usfirst.frc4904.robot.commands.indexer.IndexerSet;
import org.usfirst.frc4904.robot.commands.indexerIntakeTurret.Shoot;
import org.usfirst.frc4904.robot.commands.shooter.ShooterBrake;
import org.usfirst.frc4904.robot.commands.turret.TurretAlign;
import org.usfirst.frc4904.robot.subsystems.Indexer;
import org.usfirst.frc4904.standard.commands.RunFor;
import org.usfirst.frc4904.standard.humaninput.Operator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DefaultOperator extends Operator {
	public DefaultOperator() {
		super("DefaultOperator");
	}

	public DefaultOperator(String name) {
		super(name);
	}

	@Override
	public void bindCommands() {
		RobotMap.HumanInput.Operator.joystick.button1.whenPressed(new SequentialCommandGroup(
				new RunFor(new Shoot(RobotMap.Component.robotUDP), 4),
				new RunFor(new IndexerSet(Indexer.DEFAULT_INDEXER_SPEED, -Indexer.DEFAULT_INDEXER_SPEED), 2),
				new ShooterBrake(),
				new IndexerOff()));
		RobotMap.HumanInput.Operator.joystick.button7.whenPressed(new ClimberUp());
		RobotMap.HumanInput.Operator.joystick.button8.whenPressed(new ClimberDown());
		RobotMap.HumanInput.Operator.joystick.button9.whenPressed(new ClimberOff());
		RobotMap.HumanInput.Operator.joystick.button2.whenPressed(new TurretAlign(RobotMap.Component.robotUDP, RobotMap.Component.turret));
	}
}
