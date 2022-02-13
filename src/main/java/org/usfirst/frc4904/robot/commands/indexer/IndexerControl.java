package org.usfirst.frc4904.robot.commands.indexer; 
import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.motor.MotorConstant;
import org.usfirst.frc4904.robot.subsystems.Indexer;

public class IndexerControl extends MotorConstant {
    public IndexerControl(double Speed) {
        super(RobotMap.Component.indexer.indexerMotor1, Speed)
    }
}
