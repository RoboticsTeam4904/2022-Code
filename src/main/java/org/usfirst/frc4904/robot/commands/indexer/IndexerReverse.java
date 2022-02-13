package org.usfirst.frc4904.robot.commands.indexer;
import org.usfirst.frc4904.robot.subsystems.Indexer;
public class IndexerReverse extends IndexerSet {
    public IndexerReverse() {
        super(Indexer.DEFAULT_REVERSE_INDEXER_SPEED, Indexer.DEFAULT_REVERSE_INDEXER_SPEED);
    }
}
