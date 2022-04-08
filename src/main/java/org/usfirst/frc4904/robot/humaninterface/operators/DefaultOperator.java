package org.usfirst.frc4904.robot.humaninterface.operators;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.commands.climber.ClimberBrake;
import org.usfirst.frc4904.robot.commands.climber.ClimberDown;
import org.usfirst.frc4904.robot.commands.climber.ClimberOff;
import org.usfirst.frc4904.robot.commands.climber.ClimberUp;
import org.usfirst.frc4904.robot.commands.indexer.IndexerOff;
import org.usfirst.frc4904.robot.commands.indexer.IndexerSet;
import org.usfirst.frc4904.robot.commands.indexerIntakeTurret.Shoot;
import org.usfirst.frc4904.robot.commands.shooter.ShooterBrake;
import org.usfirst.frc4904.robot.commands.shooter.ShooterCoast;
import org.usfirst.frc4904.robot.commands.shooter.ShooterConstant;
import org.usfirst.frc4904.robot.commands.shooter.ShooterSetSpeed;
import org.usfirst.frc4904.robot.commands.turret.TurnTurret;
import org.usfirst.frc4904.robot.commands.turret.TurretAlign;
import org.usfirst.frc4904.robot.subsystems.Indexer;
import org.usfirst.frc4904.standard.LogKitten;
import org.usfirst.frc4904.standard.commands.KittenCommand;
import org.usfirst.frc4904.standard.commands.Noop;
import org.usfirst.frc4904.standard.commands.RunFor;
import org.usfirst.frc4904.standard.commands.motor.MotorBrake;
import org.usfirst.frc4904.standard.humaninput.Operator;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
				// new RunFor(new Shoot(RobotMap.Component.robotUDP), 4),
				new KittenCommand("push", LogKitten.KittenLevel.WTF),
				new ShooterCoast(),
				new RunFor(new ShooterConstant(0.5), 4),
				new KittenCommand("log log log lgo lg o glo", LogKitten.KittenLevel.WTF),
				new RunFor(new IndexerSet(Indexer.DEFAULT_INDEXER_SPEED, -Indexer.DEFAULT_INDEXER_SPEED), 2),
				new ShooterBrake(),
				new IndexerOff()));
		RobotMap.HumanInput.Operator.joystick.button7.whenPressed(new ClimberBrake());
		RobotMap.HumanInput.Operator.joystick.button2.whenPressed(new TurretAlign(RobotMap.Component.robotUDP, RobotMap.Component.turret));
		RobotMap.HumanInput.Operator.joystick.button5.whenPressed(new RunFor(new ShooterConstant(0.5), 2)); // Shoot ball at low velocity to miss goal intentionally

		RobotMap.HumanInput.Operator.joystick.button3.whenPressed(new ShooterCoast());
		RobotMap.HumanInput.Operator.joystick.button6.whenPressed(
			new ParallelCommandGroup(
				new SequentialCommandGroup(
					new RunFor(new ShooterConstant(0.5), 5), 
					new ShooterConstant(0)
				),
				new SequentialCommandGroup(
					new RunFor(new Noop(), 2), 
					new RunFor(new IndexerSet(Indexer.DEFAULT_INDEXER_SPEED, -Indexer.DEFAULT_INDEXER_SPEED), 2), 
					new IndexerOff()
				)
			)
		);
	}
}
