package org.usfirst.frc4904.robot.commands.shooter;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.LogKitten;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class GetShooterVelocity extends CommandBase {
    @Override
    public void initialize() {
        super.initialize();
    }

    public void getRate() {
        double rate = RobotMap.Component.shooterEncoder.getRate();
        LogKitten.wtf(rate);
    }
}