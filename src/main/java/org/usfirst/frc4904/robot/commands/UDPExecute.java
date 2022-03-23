package org.usfirst.frc4904.robot.commands;

import java.io.IOException;

import org.usfirst.frc4904.robot.Robot;
import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.commands.turret.TurnTurret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc4904.robot.udp.*;
import org.usfirst.frc4904.standard.LogKitten;


public class UDPExecute extends CommandBase { 
    RobotUDPClient robotUDPClient = RobotMap.Component.robotUDPClient;
    public UDPExecute(String name) {
		super();
		setName(name);
	}
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        try {
            if (robotUDPClient.goNow) {
                robotUDPClient.encode(RobotMap.Component.sensorDrive.getPose(), RobotMap.Component.navx.getVelocityX(), RobotMap.Component.navx.getVelocityY(), RobotMap.Component.turret.getAngle(), RobotMap.Component.navx.getRate());
            }
            //new TurnTurret(robotUDPClient.server.heading).schedule(false); // 0 is forwards / initial position (maybe??)
        } catch (IOException ex) {
            LogKitten.wtf("Skipped encoding " + ex.toString());
        }
        robotUDPClient.goNow = !robotUDPClient.goNow;
        
    }

    @Override
    public void end(boolean interrupted) {
        robotUDPClient.close();
    }
}
