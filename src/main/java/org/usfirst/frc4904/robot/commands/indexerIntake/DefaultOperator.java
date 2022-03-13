package org.usfirst.frc4904.robot.commands.indexerIntake;


import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.commands.FlywheelSetSpeed;
import org.usfirst.frc4904.robot.commands.intake.ExtendIntake;
import org.usfirst.frc4904.robot.commands.intake.RetractIntake;
import org.usfirst.frc4904.standard.humaninput.Operator;
import org.usfirst.frc4904.standard.commands.RunFor;
public class DefaultOperator extends Operator {
	public DefaultOperator() {
		super("DefaultOperator");
	}

	public DefaultOperator(String name) {
		super(name);
	}

	@Override
	public void bindCommands() {

    RobotMap.HumanInput.Operator.joystick.button1.whenPressed(new RunFor("FlywheelSetSpeed", new FlywheelSetSpeed(RobotMap.Component.shooter, 1), 1.5)); 
    RobotMap.HumanInput.Operator.joystick.button2.whenPressed(new RunFor(new RotateIndexerIntake(), 1.0));
//  RobotMap.HumanInput.Operator.jostick.button2.whenPressed(RunFor(CommandBase IndexerOn(), double 4));

    //RobotMap.HumanInput.Operator.jostick.button3.whenPressed(new Command());
   // RobotMap.HumanInput.Operator.jostick.button4.whenPressed(new Command());
  //  RobotMap.HumanInput.Operator.jostick.button5.whenPressed(new Command());
 //   RobotMap.HumanInput.Operator.jostick.button6.whenPressed(new Command());
    RobotMap.HumanInput.Operator.joystick.button7.whenPressed(new ExtendIntake());

    RobotMap.HumanInput.Operator.joystick.button8.whenPressed(new RetractIntake());
 //  RobotMap.HumanInput.Operator.jostick.button9.whenPressed(new Command());
  // RobotMap.HumanInput.Operator.jostick.button10.whenPressed(new Command());
   //RobotMap.HumanInput.Operator.jostick.button11.whenPressed(new Command());
    // RobotMap.HumanInput.Operator.jostick.button12.whenPressed(new Command());




	}
}