package org.usfirst.frc4904.robot.auton;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.commands.indexerIntakeTurret.Shoot;
import org.usfirst.frc4904.standard.commands.chassis.ChassisConstant;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SimpleRoutine extends SequentialCommandGroup {
    public SimpleRoutine() {
        // first backup, then shoot
        this.addCommands(
            new ChassisConstant(RobotMap.Component.chassis, 0, -1, 0, 2), // TODO: tune num of seconds, currently 2
            new Shoot(-1).withTimeout(2) // TODO: tune 
        );
    }
}
