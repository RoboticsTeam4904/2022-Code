package org.usfirst.frc4904.robot.commands.indexerIntakeTurret;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.commands.indexer.IndexerSet;
import org.usfirst.frc4904.robot.commands.intake.AxleIntakeOn;
import org.usfirst.frc4904.robot.commands.shooter.ShooterSetSpeed;
import org.usfirst.frc4904.robot.subsystems.Indexer;
import org.usfirst.frc4904.standard.commands.RunFor;



public class Shoot extends ParallelCommandGroup {
    public Shoot() {
        this.addCommands(
            new ShooterSetSpeed(((818.05 * RobotMap.Component.robotUDPClient.server.distance*RobotMap.Component.robotUDPClient.server.distance) - (639.71 * RobotMap.Component.robotUDPClient.server.distance) + 270.6))
        );
    }
}