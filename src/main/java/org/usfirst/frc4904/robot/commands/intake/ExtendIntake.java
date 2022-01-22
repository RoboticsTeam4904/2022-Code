package org.usfirst.frc4904.robot.commands.intake;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.solenoid.SolenoidExtend;

public class ExtendIntake extends SolenoidExtend { //It extends the drawbridge which has the intake mechanism attached to it.
    public ExtendIntake() {
        super("Extend Intake Drawbridge", RobotMap.Component.intakeDrawbridgeSolenoid);
    }
}