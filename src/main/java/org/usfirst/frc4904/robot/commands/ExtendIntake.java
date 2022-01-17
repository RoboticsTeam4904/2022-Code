package org.usfirst.frc4904.robot.commands;

import org.usfirst.frc4904.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtendIntake extends CommandBase {
    protected final Intake intake;
    protected double targetSpeed;
    public ExtendIntake(Intake intake, double targetSpeed) {
        super();
        this.intake = intake;
        this.targetSpeed = targetSpeed;
        
    }
}
