package org.usfirst.frc4904.robot.humaninterface.operators;


import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.commands.indexerIntakeTurret.Shoot;
import org.usfirst.frc4904.robot.commands.indexerIntakeTurret.StoreBall;
import org.usfirst.frc4904.robot.commands.intake.ExtendIntake;
import org.usfirst.frc4904.robot.commands.intake.RetractIntake;
import org.usfirst.frc4904.standard.commands.RunFor;
import org.usfirst.frc4904.standard.humaninput.Operator;

public class DefaultOperator extends Operator {
	public DefaultOperator() {
		super("DefaultOperator");
	}

	public DefaultOperator(String name) {
		super(name);
	}

	@Override
	public void bindCommands() {
		RobotMap.HumanInput.Operator.joystick.button1.whenPressed(new RunFor(new Shoot(0), 1)); // TODO: set shooter speed or create method to fetch this
		RobotMap.HumanInput.Operator.joystick.button2.whenPressed(new RunFor(new StoreBall(), 1));
		RobotMap.HumanInput.Operator.joystick.button7.whenPressed(new ExtendIntake());
		RobotMap.HumanInput.Operator.joystick.button8.whenPressed(new RetractIntake());
	}
}