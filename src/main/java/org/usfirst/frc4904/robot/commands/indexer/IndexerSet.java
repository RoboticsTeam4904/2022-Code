package org.usfirst.frc4904.robot.commands.indexer; 
import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.motor.MotorConstant;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;


public class IndexerSet extends ParallelCommandGroup {
    public IndexerSet(double holderSpeed, double beltSpeed) {
        this.addCommands(
            new MotorConstant(RobotMap.Component.indexer.holderMotor, holderSpeed),
            new MotorConstant(RobotMap.Component.indexer.beltMotor, beltSpeed)
        );
    }
}
