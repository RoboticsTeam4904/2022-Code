package org.usfirst.frc4904.robot.commands.indexerIntake;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.usfirst.frc4904.robot.commands.indexer.IndexerOn;
import org.usfirst.frc4904.robot.commands.intake.AxleIntakeOn;


public class RotateIndexerIntake extends ParallelCommandGroup {
    public RotateIndexerIntake(double speed1, double speed2) {
        this.addCommands(
            new IndexerOn(),
            new AxleIntakeOn()
        );
    }
}
