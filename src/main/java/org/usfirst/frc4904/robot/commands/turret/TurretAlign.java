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
    }

    @Override
    public void initialize() {
        super.initialize();
        final var angleGoal = turret.getAngle() + net.getLocalizationData().goalRelativeAngle()*-1;
        // final var angleGoal = Math.PI/2;
        turnCommand = new TurnTurret(angleGoal, turret);
        turnCommand.schedule();
    }

    // @Override
    // public void execute() {
    //     turnCommand.execute();
    // }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (interrupted) {
            turnCommand.end(interrupted);
        }
    }
}
