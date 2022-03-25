package org.usfirst.frc4904.robot.commands.indexerIntakeTurret;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj.RobotController; // @zbuster05 should i be worried that this is wpilibj not wpilibj2  ~ @exr0n 

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.commands.indexer.IndexerSet;
import org.usfirst.frc4904.robot.commands.intake.AxleIntakeOn;
import org.usfirst.frc4904.robot.commands.shooter.ShooterSetSpeed;
import org.usfirst.frc4904.robot.subsystems.Indexer;
import org.usfirst.frc4904.standard.commands.RunFor;



public class Shoot extends ParallelCommandGroup {
    public Shoot() {
        final double d = RobotMap.Component.robotUDPClient.server.distance;
        final double v = RobotController.getBatteryVoltage();
        // (d-29*v+383)/370 from mar 24 23:26 https://discord.com/channels/898058908915073024/898059350772441148/956801260940566589
        this.addCommands(
            new ShooterSetSpeed((d - 29*v + 383)/370)
        );
    }
}
