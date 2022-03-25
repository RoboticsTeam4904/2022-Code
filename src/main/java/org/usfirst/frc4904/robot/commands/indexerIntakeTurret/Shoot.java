package org.usfirst.frc4904.robot.commands.indexerIntakeTurret;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.commands.indexer.IndexerSet;
import org.usfirst.frc4904.robot.commands.intake.AxleIntakeOn;
import org.usfirst.frc4904.robot.commands.shooter.ShooterSetSpeed;
import org.usfirst.frc4904.robot.subsystems.Indexer;
import org.usfirst.frc4904.robot.subsystems.net.RobotUDP;
import org.usfirst.frc4904.standard.commands.RunFor;

public class Shoot extends ParallelCommandGroup {
    public Shoot(RobotUDP net) {
        this.addCommands(
                new ShooterSetSpeed((39.37 * 0.025 * net.getLocalizationData().goalDistance() + 0.01)));
    }
}
