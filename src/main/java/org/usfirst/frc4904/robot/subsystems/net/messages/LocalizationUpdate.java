package org.usfirst.frc4904.robot.subsystems.net.messages;

import java.io.IOException;

import org.msgpack.core.MessageUnpacker;
import org.usfirst.frc4904.standard.LogKitten;
import org.usfirst.frc4904.standard.subsystems.net.message.Unpackable;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class LocalizationUpdate implements Unpackable {
    private double goalDistance = 0;
    private double goalRelativeAngle = 0;

    public double goalDistance() {
        return goalDistance;
    }

    public double goalRelativeAngle() {
        return goalRelativeAngle;
    }

    @Override
    public void unpack(MessageUnpacker unpacker) throws IOException {
        unpacker.unpackArrayHeader();
        goalDistance = unpacker.unpackDouble();
        goalRelativeAngle = unpacker.unpackDouble();
        try {
            SmartDashboard.putNumber("Goal Distance", goalDistance);
            SmartDashboard.putNumber("Relative Angle", goalRelativeAngle*-1);
        } finally {}
        // LogKitten.wtf("emacs?????????????? " + goalRelativeAngle +" "+  goalDistance);
    }
}
