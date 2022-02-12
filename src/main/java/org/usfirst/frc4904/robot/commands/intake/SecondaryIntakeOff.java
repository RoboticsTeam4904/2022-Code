package org.usfirst.frc4904.robot.commands.intake;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.motor.MotorIdle;

/**
 * Stops rotating motor
 */
public class SecondaryIntakeOff extends MotorIdle {
    public SecondaryIntakeOff() {
        super("secondaryIntakeOff", RobotMap.Component.intakeSecondaryMotor);
    }
}