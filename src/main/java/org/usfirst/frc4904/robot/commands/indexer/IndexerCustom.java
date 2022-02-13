package org.usfirst.frc4904.robot.commands.indexer; 
import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.motor.MotorConstant;
import org.usfirst.frc4904.robot.subsystems.Indexer;

public class IndexerCustom extends MotorConstant {
    public IndexerCustom(double Speed1, double Speed2) {
        super("IndexerOn", RobotMap.Component.indexer.indexerMotor1, Indexer.DEFAULT_INDEXER_SPEED);
    }
}
