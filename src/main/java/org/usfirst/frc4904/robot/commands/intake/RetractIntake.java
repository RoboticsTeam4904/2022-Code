package org.usfirst.frc4904.robot.commands.intake;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.solenoid.SolenoidRetract;

public class RetractIntake extends SolenoidRetract { //It extends the drawbridge which has the intake mechanism attached to it.
    public RetractIntake() {
        super("Retract Intake Drawbridge", RobotMap.Component.intakeDrawbridgeSolenoid);
    }
}