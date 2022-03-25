package org.usfirst.frc4904.robot.commands.turret;

import org.usfirst.frc4904.robot.subsystems.Turret;
import org.usfirst.frc4904.robot.subsystems.net.RobotUDP;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public final class TurretAlign extends CommandBase {
    private final RobotUDP net;
    private final Turret turret;

    private Command turnCommand;

    public TurretAlign(RobotUDP net, Turret turret) {
        this.net = net;
        this.turret = turret;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        final var angleGoal = turret.getAngle() + net.getLocalizationData().goalRelativeAngle();

        turnCommand = new TurnTurret(angleGoal);
        turnCommand.initialize();
    }

    @Override
    public void execute() {
        turnCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return turnCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        turnCommand.end(interrupted);
    }
}