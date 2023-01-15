package org.usfirst.frc4904.robot.commands;

import org.usfirst.frc4904.standard.LogKitten;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class ParralelCommandTest extends ParallelCommandGroup {
    public ParralelCommandTest(Command... commands) {
        super(commands);
    }
    
    @Override
    public void end(boolean interrupted) {
        LogKitten.wtf("starting parallel end test " + interrupted);
        if (!interrupted) {super.end(interrupted);}
   }
}
