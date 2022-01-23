package org.usfirst.frc4904.robot.commands;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.solenoid.SolenoidExtend;

public class ExtendScissorLift extends SolenoidExtend {
    public ExtendScissorLift() {
        super("ExtendScissorLift", RobotMap.Component.scissorLift);
    }
}
