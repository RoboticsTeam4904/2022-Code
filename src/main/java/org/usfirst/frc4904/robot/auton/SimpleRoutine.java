package org.usfirst.frc4904.robot.auton;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.commands.indexer.IndexerOff;
import org.usfirst.frc4904.robot.commands.indexer.IndexerSet;
import org.usfirst.frc4904.robot.commands.indexerIntakeTurret.Shoot;
import org.usfirst.frc4904.robot.commands.intake.ExtendIntake;
import org.usfirst.frc4904.robot.commands.shooter.ShooterBrake;
import org.usfirst.frc4904.robot.commands.turret.TurnTurret;
import org.usfirst.frc4904.robot.commands.turret.TurretAlign;
import org.usfirst.frc4904.robot.subsystems.Indexer;
import org.usfirst.frc4904.robot.subsystems.Turret;
import org.usfirst.frc4904.standard.commands.RunFor;
import org.usfirst.frc4904.standard.commands.chassis.ChassisConstant;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SimpleRoutine extends SequentialCommandGroup {
    public SimpleRoutine(Turret turret) {
        // first backup, then shoot
        this.addCommands(

            new ChassisConstant(RobotMap.Component.chassis, 0, -0.5, 0, 0.5) // TODO: tune num of seconds, currently 2
            //new RunFor(new ExtendIntake(), 2.2)
            //new TurnTurret(0.0),
            //new TurretAlign(RobotMap.Component.robotUDP, RobotMap.Component.turret),
            //new RunFor(new Shoot(RobotMap.Component.robotUDP), 5),
			// new IndexerSet(Indexer.DEFAULT_INDEXER_SPEED, -Indexer.DEFAULT_INDEXER_SPEED),
			// new ShooterBrake(),
			// new IndexerOff()
        );
    }
}
