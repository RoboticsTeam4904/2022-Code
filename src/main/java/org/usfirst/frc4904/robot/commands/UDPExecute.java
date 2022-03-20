package org.usfirst.frc4904.robot.commands;

import java.io.IOException;

import org.usfirst.frc4904.robot.Robot;
import org.usfirst.frc4904.robot.RobotMap;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc4904.robot.udp.*;
import org.usfirst.frc4904.standard.LogKitten;


public class UDPExecute extends CommandBase { 
    RobotUDPClient robotUDPClient = RobotMap.Component.robotUDPClient;
    public UDPExecute(String name) {
		super();
		setName(name);
	}
    
    public void execute() {
        try {
            robotUDPClient.encode(RobotMap.Component.sensorDrive.getPose(), RobotMap.Component.navx.getRate(), RobotMap.Component.navx.getWorldLinearAccelX(), RobotMap.Component.navx.getWorldLinearAccelY(), RobotMap.Component.navx.getRate());
        } catch (IOException ex) {
            LogKitten.wtf("Skipped encoding " + ex.toString());
        }
        
    }

    public void initialize() {}
}
