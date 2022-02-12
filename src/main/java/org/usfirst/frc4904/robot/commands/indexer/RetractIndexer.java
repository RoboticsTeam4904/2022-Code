
package org.usfirst.frc4904.robot.commands.indexer;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.solenoid.SolenoidRetract;

/**
 * Retracts the drawbridge which has the intake mechanism attached to it.
 */
public class RetractIndexer extends SolenoidRetract {
    public RetractIndexer() {
        super("retractIndexer", RobotMap.Component.indexerSolenoids);
    }
}