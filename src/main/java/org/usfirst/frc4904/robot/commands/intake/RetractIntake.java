package org.usfirst.frc4904.robot.commands.intake;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.solenoid.SolenoidRetract;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/**
 * Retracts the drawbridge which has the intake mechanism attached to it.
 */
public class RetractIntake extends ParallelCommandGroup {
    public RetractIntake() {
        this.addCommands(
            new SolenoidRetract(RobotMap.Component.intakeExtender1)
        );
    }
}