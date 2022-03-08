package org.usfirst.frc4904.robot.commands.indexerIntakeTurret;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.usfirst.frc4904.robot.commands.indexer.IndexerSet;
import org.usfirst.frc4904.robot.commands.intake.AxleIntakeOn;
import org.usfirst.frc4904.robot.subsystems.Indexer;
import org.usfirst.frc4904.robot.commands.FlywheelSetSpeed;



public class Shoot extends ParallelCommandGroup {
    public Shoot(double speed1, double speed2) {
        this.addCommands(
            new AxleIntakeOn(),
            new IndexerSet(Indexer.DEFAULT_INDEXER_SPEED, Indexer.DEFAULT_INDEXER_SPEED),
            new FlywheelSetSpeed(-1) //TODO: Set speed
        );
    }
}