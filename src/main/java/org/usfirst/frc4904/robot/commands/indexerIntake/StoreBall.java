package org.usfirst.frc4904.robot.commands.indexerIntake;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.usfirst.frc4904.robot.commands.indexer.IndexerSet;
import org.usfirst.frc4904.robot.commands.intake.AxleIntakeOn;
import org.usfirst.frc4904.robot.subsystems.Indexer;


public class RotateIndexerIntake extends ParallelCommandGroup {
    public RotateIndexerIntake(double speed1, double speed2) {
        this.addCommands(
            new IndexerSet(DEFAULT_INDEXER_SPEED, DEFAULT_REVERSE_INDEXER_SPEED),
            new AxleIntakeOn()
        );
    }
}
