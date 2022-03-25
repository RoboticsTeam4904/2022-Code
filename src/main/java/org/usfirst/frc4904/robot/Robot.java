/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc4904.robot;

import org.usfirst.frc4904.robot.auton.SimpleRoutine;
import org.usfirst.frc4904.robot.commands.UDPExecute;
import org.usfirst.frc4904.robot.commands.turret.TurnTurret;
import org.usfirst.frc4904.robot.humaninterface.drivers.NathanGain;
import org.usfirst.frc4904.robot.humaninterface.operators.DefaultOperator;
import org.usfirst.frc4904.standard.CommandRobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import org.usfirst.frc4904.standard.commands.KittenCommand;

import org.usfirst.frc4904.standard.LogKitten;

public class Robot extends CommandRobotBase {
    private RobotMap map = new RobotMap();
    private UDPExecute udpExecute;

    @Override
    public void initialize() {
        driverChooser.setDefaultOption(new NathanGain());
        operatorChooser.setDefaultOption(new DefaultOperator());
    }

    @Override
    public void teleopInitialize() {
        // udpExecute = new UDPExecute("UDPExecute");
        // udpExecute.schedule(false);
    }

    @Override
    public void teleopExecute() {

    }

    @Override
    public void autonomousInitialize() {
        udpExecute = new UDPExecute("UDPExecute");
        udpExecute.schedule(false);
        CommandGroupBase routine = new SimpleRoutine();
        routine.schedule();
    }

    @Override
    public void autonomousExecute() {
    }

    @Override
    public void disabledInitialize() {
    }

    @Override
    public void disabledExecute() {
    }

    @Override
    public void testInitialize() {
    }

    @Override
    public void testExecute() {
    }

    @Override
    public void alwaysExecute() {
    }

}