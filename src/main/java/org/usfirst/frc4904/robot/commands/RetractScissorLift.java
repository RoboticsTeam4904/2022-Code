package org.usfirst.frc4904.robot.commands;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.solenoid.SolenoidRetract;

public class RetractScissorLift extends SolenoidRetract {
    public RetractScissorLift() {
        super("RetractScissorLift", RobotMap.Component.scissorLift);
    }
}
