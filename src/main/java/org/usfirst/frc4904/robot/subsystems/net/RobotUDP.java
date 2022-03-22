package org.usfirst.frc4904.robot.subsystems.net;

import java.io.IOException;
import java.net.SocketAddress;

import org.msgpack.core.MessageUnpacker;
import org.usfirst.frc4904.robot.subsystems.net.messages.LocalizationUpdate;
import org.usfirst.frc4904.robot.subsystems.net.messages.OdometryUpdate;
import org.usfirst.frc4904.standard.LogKitten;
import org.usfirst.frc4904.standard.subsystems.net.UDPSocket;

public class RobotUDP extends UDPSocket {
    private SocketAddress localizationAddr;

    private LocalizationUpdate localizationData;

    public RobotUDP(SocketAddress localAddr, SocketAddress localizationAddr) throws IOException {
        super(localAddr);

        this.localizationAddr = localizationAddr;
    }

    @Override
    protected void receive(SocketAddress address, MessageUnpacker unpacker) throws IOException {
        if (address.equals(localizationAddr)) {
            final var update = new LocalizationUpdate();
            update.unpack(unpacker);

            localizationData = update;
        }

        LogKitten.w("Received message from unexpected address: " + address);
    }

    public void updateOdometry(OdometryUpdate message) {
        try {
            send(localizationAddr, message);
        } catch (IOException ex) {
            LogKitten.ex(ex);
        }
    }

    public LocalizationUpdate getLocalizationData() {
        return localizationData;
    }
}
