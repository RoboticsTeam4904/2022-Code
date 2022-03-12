package org.usfirst.frc4904.robot.commands.indexerIntakeTurret;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.usfirst.frc4904.robot.commands.indexer.IndexerSet;
import org.usfirst.frc4904.robot.commands.intake.AxleIntakeOn;
import org.usfirst.frc4904.robot.subsystems.Indexer;

public class StoreBall extends ParallelCommandGroup {
    public StoreBall() {
        this.addCommands(
            new IndexerSet(Indexer.DEFAULT_INDEXER_SPEED, Indexer.DEFAULT_REVERSE_INDEXER_SPEED),
            new AxleIntakeOn()
        );
    }
}
