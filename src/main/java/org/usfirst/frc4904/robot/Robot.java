/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc4904.robot;

import org.usfirst.frc4904.robot.auton.SimpleRoutine;
import org.usfirst.frc4904.robot.commands.net.OdometrySend;
import org.usfirst.frc4904.robot.commands.turret.TurnTurret;
import org.usfirst.frc4904.robot.humaninterface.drivers.NathanGain;
import org.usfirst.frc4904.robot.humaninterface.operators.DefaultOperator;
import org.usfirst.frc4904.standard.CommandRobotBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import org.usfirst.frc4904.standard.commands.KittenCommand;

import org.usfirst.frc4904.standard.LogKitten;

public class Robot extends CommandRobotBase {
    private RobotMap map = new RobotMap();

    @Override
    public void initialize() {
        driverChooser.setDefaultOption(new NathanGain());
        operatorChooser.setDefaultOption(new DefaultOperator());
        RobotMap.Component.sensorDrive.resetOdometry(new Pose2d());
    }

    @Override
    public void teleopInitialize() {
    }

    @Override
    public void teleopExecute() {
    }

    @Override
    public void autonomousInitialize() {
        final var routine = new SimpleRoutine();
        routine.schedule();

        final var odometrySend = new OdometrySend(
                RobotMap.Component.robotUDP,
                RobotMap.Component.sensorDrive,
                RobotMap.Component.navx,
                RobotMap.Component.turret);

        odometrySend.schedule(false);
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
        final var localizationData = RobotMap.Component.robotUDP.getLocalizationData();

        RobotMap.NetworkTables.Localization.goalDistance.setDouble(localizationData.goalDistance());
        RobotMap.NetworkTables.Localization.goalRelativeAngle.setDouble(localizationData.goalRelativeAngle());

        final var odometryData = RobotMap.Component.robotUDP.getOdometryData();

        final var pose = odometryData.pose().pose();

        RobotMap.NetworkTables.Odometry.pose.setDoubleArray(new double[] {
                pose.getRotation().getRadians(),
                pose.getX(),
                pose.getY(),
        });

        final var accel = odometryData.accel().pose();

        RobotMap.NetworkTables.Odometry.accel.setDoubleArray(new double[] {
                accel.getRotation().getRadians(),
                accel.getX(),
                accel.getY(),
        });

        RobotMap.NetworkTables.Odometry.turretAngle.setDouble(odometryData.turretAngle());
    }

}
