package org.usfirst.frc4904.robot.subsystems.net.messages;

import java.io.IOException;

import org.msgpack.core.MessagePacker;
import org.usfirst.frc4904.standard.subsystems.net.message.Packable;

import edu.wpi.first.math.geometry.Pose2d;

public final class OdometryUpdate implements Packable {
    private PoseData pose;
    private PoseData accel;
    private double turretAngle;
    private long timestamp;

    public OdometryUpdate(Pose2d pose, Pose2d accel, double turretAngle, long timestamp) {
        this.pose = new PoseData(pose);
        this.accel = new PoseData(accel);
        this.timestamp = timestamp;
    }

    public PoseData pose() {
        return pose;
    }

    public PoseData accel() {
        return accel;
    }

    public double turretAngle() {
        return turretAngle;
    }

    public long timestamp() {
        return timestamp;
    }

    @Override
    public void pack(MessagePacker packer) throws IOException {
        packer.packArrayHeader(2);

        packer.packArrayHeader(3);
        pose.pack(packer);
        accel.pack(packer);
        packer.packDouble(turretAngle);

        packer.packLong(timestamp);
    }
}
