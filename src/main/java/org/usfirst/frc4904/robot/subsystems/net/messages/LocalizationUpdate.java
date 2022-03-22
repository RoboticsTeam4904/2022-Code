package org.usfirst.frc4904.robot.subsystems.net.messages;

import java.io.IOException;

import org.msgpack.core.MessageUnpacker;
import org.usfirst.frc4904.standard.subsystems.net.message.Unpackable;

public final class LocalizationUpdate implements Unpackable {
    public double goalDistance;
    public double goalRelativeAngle;

    @Override
    public void unpack(MessageUnpacker unpacker) throws IOException {
        goalDistance = unpacker.unpackDouble();
        goalRelativeAngle = unpacker.unpackDouble();
    }
}
