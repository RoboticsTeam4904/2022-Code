package org.usfirst.frc4904.robot.commands.net;

import java.util.concurrent.TimeUnit;

import org.usfirst.frc4904.robot.subsystems.Turret;
import org.usfirst.frc4904.robot.subsystems.net.RobotUDP;
import org.usfirst.frc4904.robot.subsystems.net.messages.OdometryUpdate;
import org.usfirst.frc4904.standard.Util;
import org.usfirst.frc4904.standard.custom.sensors.NavX;
import org.usfirst.frc4904.standard.subsystems.chassis.SensorDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class OdometrySend extends CommandBase {
    private final static double SECS_PER_MICROSEC = Util.timeConversionFactor(TimeUnit.MICROSECONDS, TimeUnit.SECONDS);

    private final RobotUDP net;

    private final SensorDrive sensorDrive;
    private final NavX navx;
    private final Turret turret;

    private long lastTimestamp;

    public OdometrySend(RobotUDP net, SensorDrive sensorDrive, NavX navx, Turret turret) {
        this.net = net;

        this.sensorDrive = sensorDrive;
        this.navx = navx;
        this.turret = turret;
    }

    @Override
    public void execute() {
        final long timestamp = RobotController.getFPGATime();

        final double dt = lastTimestamp == 0 ? 0 : (timestamp - lastTimestamp) * SECS_PER_MICROSEC;
        final var accelYawDegrees = dt == 0 ? 0 : navx.getRate() / dt;

        final var accel = new Pose2d(
                navx.getWorldLinearAccelX(),
                navx.getWorldLinearAccelY(),
                Rotation2d.fromDegrees(accelYawDegrees));

        final var update = new OdometryUpdate(
                sensorDrive.getPose(),
                accel,
                turret.getAngle(),
                timestamp);

        net.updateOdometry(update);
        lastTimestamp = timestamp;
    }
}
