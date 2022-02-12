package org.usfirst.frc4904.robot.commands.indexer;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.solenoid.SolenoidExtend;

/**
 * Extends the drawbridge which has the intake mechanism attached to it.
 */
public class ExtendIndexer extends SolenoidExtend {
    public ExtendIndexer() {
        super("extendIndexer", RobotMap.Component.indexerSolenoids);
    }
}