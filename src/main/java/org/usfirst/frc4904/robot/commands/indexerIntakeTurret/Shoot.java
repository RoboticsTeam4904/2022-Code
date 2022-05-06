package org.usfirst.frc4904.robot.commands.indexerIntakeTurret;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj.RobotController;

import org.usfirst.frc4904.robot.commands.indexer.IndexerSet;
import org.usfirst.frc4904.robot.commands.intake.AxleIntakeOn;
import org.usfirst.frc4904.robot.commands.shooter.ShooterSetSpeed;
import org.usfirst.frc4904.robot.subsystems.Indexer;
import org.usfirst.frc4904.robot.subsystems.net.RobotUDP;
import org.usfirst.frc4904.standard.commands.RunFor;

public class Shoot extends ParallelCommandGroup {
    public Shoot(RobotUDP net) {
        final double d = net.getLocalizationData().goalDistance();
        final double v = RobotController.getBatteryVoltage();

        // final double d = 150;
        // from mar 24 23:26
        // https://discord.com/channels/898058908915073024/898059350772441148/956801260940566589
        final double speed = (d - (29 * v) + 383) / 370; // TODO cringe

        // this.addCommands(new ShooterSetSpeed(0));
        this.addCommands(new ShooterSetSpeed(speed));
    }
}
