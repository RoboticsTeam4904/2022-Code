package org.usfirst.frc4904.robot.commands.intake;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.motor.MotorIdle;

public class AxleIntakeOff extends MotorIdle {
    public AxleIntakeOff() {
        super("Stop Axle Intake", RobotMap.Component.intakeAxleMotor);
    }
}