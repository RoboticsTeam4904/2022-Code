/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc4904.robot;

import org.usfirst.frc4904.robot.auton.SimpleRoutine;
import org.usfirst.frc4904.robot.commands.climber.ClimberBrake;
import org.usfirst.frc4904.robot.commands.climber.ClimberDown;
import org.usfirst.frc4904.robot.commands.climber.ClimberOff;
import org.usfirst.frc4904.robot.commands.climber.ClimberUp;
import org.usfirst.frc4904.robot.commands.net.OdometrySend;
import org.usfirst.frc4904.robot.commands.turret.TurnTurret;
import org.usfirst.frc4904.robot.commands.turret.TurretMotorConstant;
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
        final var odometrySend = new OdometrySend(
            RobotMap.Component.robotUDP,
            RobotMap.Component.sensorDrive,
            RobotMap.Component.navx,
            RobotMap.Component.turret);

        odometrySend.schedule();

        RobotMap.Component.turretMotor.setCoastMode();
    }

    @Override
    public void teleopExecute() {
        double zgain = RobotMap.HumanInput.Operator.joystick.getZ() * 2;
        if (zgain != 0) {
            new TurretMotorConstant(Math.pow(Math.abs(zgain), 0.8) * -0.1 * Math.signum(zgain)).schedule();
        }

        // if (RobotMap.HumanInput.Operator.joystick.getAxis(3) < -0.95) {
        //     new ClimberDown().schedule();
        // } else if (RobotMap.HumanInput.Operator.joystick.getAxis(3) > 0.95) {
        //     new ClimberUp().schedule();
        // } else {
        //     new ClimberOff().schedule();
        // }

        // LogKitten.wtf(RobotMap.Component.shooterTalon.get());
    }

    @Override
    public void autonomousInitialize() {
        final var routine = new SimpleRoutine(RobotMap.Component.turret);
        routine.schedule();

        final var odometrySend = new OdometrySend(
                RobotMap.Component.robotUDP,
                RobotMap.Component.sensorDrive,
                RobotMap.Component.navx,
                RobotMap.Component.turret);

        odometrySend.schedule();

        RobotMap.Component.turretMotor.setCoastMode();
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

        final var odometryData = RobotMap.Component.robotUDP.getOdometryData();

        // final var pose = odometryData.pose().pose();

        // RobotMap.NetworkTables.Odometry.pose.setDoubleArray(new double[] {
        //         pose.getRotation().getRadians(),
        //         pose.getX(),
        //         pose.getY(),
        // });

        // final var accel = odometryData.accel().pose();
    }

}
