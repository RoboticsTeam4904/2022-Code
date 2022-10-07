package org.usfirst.frc4904.robot.commands.indexerIntakeTurret;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.usfirst.frc4904.robot.commands.indexer.IndexerSet;
import org.usfirst.frc4904.robot.commands.intake.AxleIntakeOn;
import org.usfirst.frc4904.robot.subsystems.Indexer;

public class StoreBall extends ParallelCommandGroup {
    public StoreBall() {
        this.addCommands(
            new IndexerSet(Indexer.DEFAULT_INTAKE_SPEED*0.5, Indexer.DEFAULT_INTAKE_SPEED * 1.3),
            new AxleIntakeOn()
        );
    }
}
