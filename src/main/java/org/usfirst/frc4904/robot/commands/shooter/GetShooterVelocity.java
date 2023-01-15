package org.usfirst.frc4904.robot.commands.shooter;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.LogKitten;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class GetShooterVelocity extends CommandBase {
    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        getRate();
    }

    public void getRate() {
        double rate = RobotMap.Component.shooterEncoder.getRate();
        double godlnows_voltage = RobotMap.Component.shooterTalon.getMotorOutputVoltage();
        double godeelnows_voltage = RobotMap.Component.shooterTalon.getBusVoltage();
        double godlnows_voooltage = RobotMap.Component.shooterTalon.getSelectedSensorVelocity();
        LogKitten.wtf("eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee    " + rate + godlnows_voltage + godeelnows_voltage + godlnows_voooltage + RobotMap.Component.shooterTalon.getClosedLoopError());
    }
}