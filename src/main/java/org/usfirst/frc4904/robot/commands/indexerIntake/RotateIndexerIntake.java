package org.usfirst.frc4904.robot.commands.indexerIntake;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.motor.MotorConstant;
import org.usfirst.frc4904.robot.subsystems.Indexer;
import org.usfirst.frc4904.robot.commands.intake.AxleIntakeOn;


public class RotateIndexerIntake extends ParallelCommandGroup {
    public RotateIndexerIntake(double speed1, double speed2) {
        this.addCommands(
            new MotorConstant(RobotMap.Component.indexer.indexerMotor1, Indexer.DEFAULT_INDEXER_SPEED),
            new MotorConstant(RobotMap.Component.indexer.indexerMotor2, Indexer.DEFAULT_INDEXER_SPEED),
            new MotorConstant(RobotMap.Component.intakeAxleMotor, AxleIntakeOn.DEFAULT_INTAKE_MOTOR_SPEED)
        );
    }
}
