package org.usfirst.frc4904.robot.humaninterface.operators;

import java.util.List;

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
import org.usfirst.frc4904.standard.LogKitten.KittenLevel;
import org.usfirst.frc4904.standard.commands.KittenCommand;
import org.usfirst.frc4904.standard.commands.Noop;
import org.usfirst.frc4904.standard.commands.RunFor;
import org.usfirst.frc4904.standard.commands.chassis.SimpleSplines;
import org.usfirst.frc4904.standard.commands.motor.MotorBrake;
import org.usfirst.frc4904.standard.humaninput.Operator;
import org.usfirst.frc4904.standard.subsystems.chassis.SplinesDrive;
import org.usfirst.frc4904.robot.commands.ParralelCommandTest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class DefaultOperator extends Operator {
	public DefaultOperator() {
		super("DefaultOperator");
	}

	public DefaultOperator(String name) {
		super(name);
	}

	@Override
	public void bindCommands() {
		// RobotMap.HumanInput.Operator.joystick.button1.whenPressed(new SequentialCommandGroup(
		// 		// new RunFor(new Shoot(RobotMap.Component.robotUDP), 4),
		// 		new KittenCommand("push", LogKitten.KittenLevel.WTF),
		// 		new ShooterCoast(),
		// 		new RunFor(new ShooterConstant(0.5), 4),
		// 		new KittenCommand("log log log lgo lg o glo", LogKitten.KittenLevel.WTF),
		// 		new RunFor(new IndexerSet(Indexer.DEFAULT_INDEXER_SPEED, -Indexer.DEFAULT_INDEXER_SPEED), 2),
		// 		new ShooterBrake(),
		// 		new IndexerOff()));
		// RobotMap.HumanInput.Operator.joystick.button7.whenPressed(new ClimberBrake());
		// RobotMap.HumanInput.Operator.joystick.button2.whileHeld(new TurretAlign(RobotMap.Component.robotUDP, RobotMap.Component.turret));
		// RobotMap.HumanInput.Operator.joystick.button12.whenPressed(new TurnTurret(Math.PI / 2, RobotMap.Component.turret));
		// RobotMap.HumanInput.Operator.joystick.button11.whenPressed(new TurnTurret(0, RobotMap.Component.turret));
		RobotMap.HumanInput.Operator.joystick.button5.whenPressed(new ParallelCommandGroup(
			new SequentialCommandGroup(
			new WaitCommand(1),
				new RunFor(new IndexerSet(Indexer.DEFAULT_INDEXER_SPEED, -Indexer.DEFAULT_INDEXER_SPEED), 2), 
				new IndexerOff()
			),
			new RunFor(new ShooterConstant(0.2), 3)
		)); // Shoot ball at low velocity to miss goal intentionally

		RobotMap.HumanInput.Operator.joystick.button3.whenPressed(new ShooterCoast());
		RobotMap.HumanInput.Operator.joystick.button1.whenPressed(
			new ParralelCommandTest(
				new SequentialCommandGroup(
					new WaitCommand(5),
					new KittenCommand("emacs1skdvjbskjdvnksjdvnkjsbdvkjsbdvkjsbdvkjbsdvkjbsdkvjbsdk\nemacs1skdvjbskjdvnksjdvnkjsbdvkjsbdvkjsbdvkjbsdvkjbsdkvjbsdk\nemacs1skdvjbskjdvnksjdvnkjsbdvkjsbdvkjsbdvkjbsdvkjbsdkvjbsdk\nemacs1skdvjbskjdvnksjdvnkjsbdvkjsbdvkjsbdvkjbsdvkjbsdkvjbsdk\n", KittenLevel.WTF),
					new KittenCommand("emacs1", KittenLevel.WTF),
					new RunFor(new IndexerSet(Indexer.DEFAULT_INDEXER_SPEED, -Indexer.DEFAULT_INDEXER_SPEED*1.5), 5), 
					// new IndexerOff(),
					new KittenCommand("emacs1.5", KittenLevel.WTF),
					new WaitCommand(1),
					new KittenCommand("emacs2", KittenLevel.WTF),

					new RunFor(new IndexerSet(Indexer.DEFAULT_INDEXER_SPEED+0.27, -Indexer.DEFAULT_INDEXER_SPEED-0.27), 0.1), 
					new IndexerOff(),
					new KittenCommand("emacs3", KittenLevel.WTF),

					new WaitCommand(10)
				),
				new RunFor(new ShooterConstant(0.48), 100)
			)
		);

		RobotMap.HumanInput.Operator.joystick.button7.whenPressed(new ClimberUp());
		RobotMap.HumanInput.Operator.joystick.button7.whenReleased(new ClimberOff());

		RobotMap.HumanInput.Operator.joystick.button8.whenPressed(new ClimberDown());
		RobotMap.HumanInput.Operator.joystick.button8.whenReleased(new ClimberOff());
		RobotMap.HumanInput.Operator.joystick.button12.whenPressed(new SimpleSplines(RobotMap.Component.SplinesDrive,RobotMap.Component.SplinesDrive.getPose(), List.of(new Translation2d(1, 1), new Translation2d(2, -1)), new Pose2d(3, 0, new Rotation2d(0)),3)); //change max voltage
	}
}
