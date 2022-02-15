package org.usfirst.frc4904.robot.commands.intake;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.solenoid.SolenoidRetract;

/**
 * Retracts the drawbridge which has the intake mechanism attached to it.
 */
public class RetractIntake extends SolenoidRetract {
    public RetractIntake() {
        super("retractIntakeDrawbridge", RobotMap.Component.intakeDrawbridgeSolenoid);
    }
}