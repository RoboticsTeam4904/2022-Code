package org.usfirst.frc4904.robot.udp;

//TODO fix imports
import org.msgpack.core.MessagePack;
import org.msgpack.core.MessagePack.PackerConfig;
import org.msgpack.core.MessagePack.UnpackerConfig;
import org.msgpack.core.MessageBufferPacker;
import org.msgpack.core.MessageFormat;
import org.msgpack.core.MessagePacker;
import org.msgpack.core.MessageUnpacker;
import org.msgpack.value.ArrayValue;
import org.msgpack.value.ExtensionValue;
import org.msgpack.value.FloatValue;
import org.msgpack.value.IntegerValue;
import org.msgpack.value.TimestampValue;
import org.msgpack.value.Value;
import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.udp.Client;
import org.usfirst.frc4904.standard.udp.Server;
import java.util.concurrent.TimeUnit;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;

import java.io.*;
import java.util.HashMap;

public class RobotUDPClient {
    Client client;
    RobotUDPServer server;
    
    public int receivingSocketNum = RobotMap.Port.UDPPorts.receivingUDPSocket;
    public int sendingSocketNum = RobotMap.Port.UDPPorts.sendingUDPSocket;

    public void setup() {
        System.out.println("Setting up sending on socket #" + sendingSocketNum + ".");
        System.out.println("Setting up sending on socket #" + receivingSocketNum + ".");

        try {
            server = new RobotUDPServer(receivingSocketNum);
            server.start();
            client = new Client("CLIENT##", sendingSocketNum);
        } catch (IOException ex) {
            System.out.println("ERR: IOException during setup. This error is from creating the Server.");
            ex.printStackTrace();
        }
    }

    public void encode(Pose2d pos, double accl_x, double accl_y, double accl_gyro, double turret_angle) throws IOException {
        MessageBufferPacker packer = MessagePack.newDefaultBufferPacker();
        // x position, odometry y position, gyro angle, x accel, y accel, gyro acceleration, turrent angle, time 
        packer
            .packDouble(pos.getX())
            .packDouble(pos.getY())
            .packDouble(pos.getRotation().getRadians())
            .packDouble(accl_x)
            .packDouble(accl_y)
            .packDouble(accl_gyro)
            .packDouble(turret_angle)
            .packDouble(Timer.getFPGATimestamp());
        packer.close();
        client.sendGenericEcho(packer);
        client.close();
    }
}