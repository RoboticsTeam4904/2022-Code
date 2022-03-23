package org.usfirst.frc4904.robot.subsystems.net.messages;

import java.io.IOException;

import org.msgpack.core.MessagePacker;
import org.usfirst.frc4904.standard.subsystems.net.message.Packable;

import edu.wpi.first.math.geometry.Pose2d;

public final class PoseData implements Packable {
    private Pose2d pose;

    public PoseData(Pose2d pose) {
        this.pose = pose;
    }

    public Pose2d pose() {
        return pose;
    }

    private void packRotation(MessagePacker packer) throws IOException {
        packer.packDouble(pose.getRotation().getRadians());
    }

    private void packTranslation(MessagePacker packer) throws IOException {
        packer
                .packArrayHeader(2)
                .packDouble(pose.getX())
                .packDouble(pose.getY());
    }

    @Override
    public void pack(MessagePacker packer) throws IOException {
        packer.packArrayHeader(2);

        packRotation(packer);
        packTranslation(packer);
    }
}
