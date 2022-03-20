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
import org.usfirst.frc4904.robot.Robot;
import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.udp.Client;
import org.usfirst.frc4904.standard.udp.Server;
import java.util.concurrent.TimeUnit;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.geometry.Pose2d;

import java.io.*;
import java.util.HashMap;

public class RobotUDPClient {
    Client client;
    public RobotUDPServer server;
    
    public int receivingSocketNum = RobotMap.Port.UDPPorts.receivingUDPSocket;
    public int sendingSocketNum = RobotMap.Port.UDPPorts.sendingUDPSocket;
    public int sourcePort = RobotMap.Port.UDPPorts.sourcePort;

    public String nanoHostname = RobotMap.Port.UDPPorts.nanoHostname;

    public void setup() {
        System.out.println("Setting up sending on socket #" + sendingSocketNum + ".");
        System.out.println("Setting up sending on socket #" + receivingSocketNum + ".");

        try {
            server = new RobotUDPServer(receivingSocketNum);
            server.start();
            client = new Client(nanoHostname, sourcePort, sendingSocketNum);
        } catch (IOException ex) {
            System.out.println("ERR: IOException during setup. This error is from creating the Server.");
            ex.printStackTrace();
        }
    }

    public void encode(Pose2d pos, double accl_x, double accl_y, double accl_gyro, double turret_angle) throws IOException {
        MessageBufferPacker packer = MessagePack.newDefaultBufferPacker();
        // x position, odometry y position, gyro angle, x accel, y accel, gyro acceleration, turrent angle, time 
        //
        packer
            .packArrayHeader(2)
            .packArrayHeader(3)
            .packArrayHeader(2)
            .packDouble(pos.getRotation().getRadians())
            .packArrayHeader(2)
            .packDouble(pos.getX())
            .packDouble(pos.getY())
            .packArrayHeader(2)
            .packDouble(accl_gyro)
            .packArrayHeader(2)
            .packDouble(accl_x)
            .packDouble(accl_y)
            .packDouble(turret_angle)
            .packLong(RobotController.getFPGATime());
        packer.close();
        client.sendGenericEcho(packer);
        client.close();
    }
}