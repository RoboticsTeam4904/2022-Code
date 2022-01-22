package org.usfirst.frc4904.robot.commands.intake;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.solenoid.SolenoidExtend;

/**
 * Extends the drawbridge which has the intake mechanism attached to it.
 */
public class ExtendIntake extends SolenoidExtend {
    public ExtendIntake() {
        super("extendIntakeDrawbridge", RobotMap.Component.intakeDrawbridgeSolenoid);
    }
}