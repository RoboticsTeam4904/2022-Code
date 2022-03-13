/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc4904.robot;

import org.usfirst.frc4904.robot.commands.turret.TurnTurret;
import org.usfirst.frc4904.standard.CommandRobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import org.usfirst.frc4904.standard.LogKitten;

public class Robot extends CommandRobotBase {
    private RobotMap map = new RobotMap();

    @Override
    public void initialize() {
    }

    @Override
    public void teleopInitialize() {
    }

    @Override
    public void teleopExecute() {
        Command turretControl = new TurnTurret(RobotMap.HumanInput.Operator.joystick.getX() / 10.0);
        turretControl.execute();
    }

    @Override
    public void autonomousInitialize() {
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