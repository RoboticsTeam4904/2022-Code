package org.usfirst.frc4904.robot.auton;

import java.time.Duration;
import java.util.concurrent.TimeUnit;

import org.usfirst.frc4904.robot.commands.indexerIntakeTurret.Shoot;
import org.usfirst.frc4904.robot.commands.turret.TurretAlign;
import org.usfirst.frc4904.robot.subsystems.Turret;
import org.usfirst.frc4904.robot.subsystems.net.RobotUDP;
import org.usfirst.frc4904.standard.commands.RunIf;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class ShootRoutine extends SequentialCommandGroup {
    private static final Duration WAIT_BEFORE_SHOOT = Duration.ofMillis(500);
    private static final double MAX_RELATIVE_ANGLE_THRESHOLD = Math.toRadians(5);

    private final RobotUDP net;
    private final Turret turret;

    public ShootRoutine(RobotUDP net, Turret turret) {
        this.net = net;
        this.turret = turret;

        addCommands(
                new TurretAlign(net, turret),
                new WaitCommand(WAIT_BEFORE_SHOOT.toMillis() / 1000.0).andThen(
                        new RunIf(new Shoot(net), this::shouldShoot)));
    }

    // TODO: This would probably be better served by a subsystem with the job of
    // tracking targets, but this should suffice for now.
    protected boolean shouldShoot() {
        return net.getLocalizationData().goalRelativeAngle() <= MAX_RELATIVE_ANGLE_THRESHOLD;
    }
}
