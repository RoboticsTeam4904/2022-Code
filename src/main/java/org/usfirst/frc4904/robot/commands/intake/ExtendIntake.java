package org.usfirst.frc4904.robot.commands.intake;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.RunFor;
import org.usfirst.frc4904.standard.commands.solenoid.SolenoidExtend;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/**
 * Extends the drawbridge which has the intake mechanism attached to it.
 */
public class ExtendIntake extends ParallelCommandGroup {
    public ExtendIntake() {
        this.addCommands(
            new SolenoidExtend(RobotMap.Component.intakeExtender1),
            new RunFor(new AxleIntakeOn(), 1)
        );
    }
}