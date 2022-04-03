package org.usfirst.frc4904.robot.commands.indexerIntakeTurret;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.usfirst.frc4904.robot.commands.indexer.IndexerOff;
import org.usfirst.frc4904.robot.commands.intake.AxleIntakeOff;


public class IndexerIntakeOff extends ParallelCommandGroup {
    public IndexerIntakeOff() {
        this.addCommands(
            new IndexerOff(),
            new AxleIntakeOff()
        );
    }
}
