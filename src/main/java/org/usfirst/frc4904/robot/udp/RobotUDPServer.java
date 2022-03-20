package org.usfirst.frc4904.robot.udp;

import java.io.IOException;

import org.usfirst.frc4904.standard.udp.Server;

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

import java.io.*;
import java.util.HashMap;


public class RobotUDPServer extends Server {
    public double distance;
    public double heading;

    RobotUDPServer(int SocketNum) throws IOException {
        super(SocketNum, RobotMap.Port.UDPPorts.nanoHostname);
    }

    protected void decode(byte[] data) throws IOException {
        MessageUnpacker unpacker = MessagePack.newDefaultUnpacker(data);
        distance = unpacker.unpackDouble();
        heading  = unpacker.unpackDouble();
        unpacker.close();
    }
}
