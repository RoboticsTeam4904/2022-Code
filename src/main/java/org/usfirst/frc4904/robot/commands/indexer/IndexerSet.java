package org.usfirst.frc4904.robot.commands.indexer; 
import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.motor.MotorConstant;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;


public class IndexerSet extends ParallelCommandGroup {
    public IndexerSet(double speed1, double speed2) {
        this.addCommands(
            new MotorConstant(RobotMap.Component.indexer.indexerMotor1, speed1),
            new MotorConstant(RobotMap.Component.indexer.indexerMotor2, speed2)
        );
    }
}
