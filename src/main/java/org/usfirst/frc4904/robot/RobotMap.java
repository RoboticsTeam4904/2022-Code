package org.usfirst.frc4904.robot;

import org.usfirst.frc4904.standard.custom.controllers.CustomJoystick;
import org.usfirst.frc4904.standard.custom.controllers.CustomXbox;
import org.usfirst.frc4904.standard.custom.motioncontrollers.CANTalonFX;
import org.usfirst.frc4904.standard.subsystems.motor.Motor;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.SocketAddress;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import org.usfirst.frc4904.robot.commands.shooter.ShooterBrake;
import org.usfirst.frc4904.robot.humaninterface.drivers.NathanGain;
import org.usfirst.frc4904.robot.subsystems.Indexer;
import org.usfirst.frc4904.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import org.usfirst.frc4904.standard.LogKitten;
import org.usfirst.frc4904.standard.commands.chassis.ChassisMove;
import org.usfirst.frc4904.standard.custom.PCMPort;
import org.usfirst.frc4904.standard.custom.motioncontrollers.CANTalonSRX;
import org.usfirst.frc4904.standard.custom.motioncontrollers.CustomPIDController;
import org.usfirst.frc4904.standard.subsystems.SolenoidSubsystem.SolenoidState;
import org.usfirst.frc4904.standard.subsystems.SolenoidSubsystem;
import org.usfirst.frc4904.robot.subsystems.Climber;

import org.usfirst.frc4904.standard.subsystems.chassis.TankDrive;
import org.usfirst.frc4904.standard.subsystems.chassis.TankDriveShifting;
import org.usfirst.frc4904.standard.custom.sensors.EncoderPair;
import org.usfirst.frc4904.standard.custom.sensors.CANTalonEncoder;
import edu.wpi.first.wpilibj.SerialPort;
import org.usfirst.frc4904.standard.subsystems.chassis.TankDrive;
import org.usfirst.frc4904.standard.subsystems.motor.Motor;
import org.usfirst.frc4904.standard.custom.sensors.NavX;

import org.usfirst.frc4904.standard.subsystems.chassis.SensorDrive;
import org.usfirst.frc4904.standard.subsystems.chassis.SolenoidShifters;
import org.usfirst.frc4904.standard.subsystems.motor.PositionSensorMotor;
import org.usfirst.frc4904.standard.subsystems.motor.VelocitySensorMotor;
import org.usfirst.frc4904.robot.subsystems.Turret;
import org.usfirst.frc4904.robot.subsystems.net.RobotUDP;

public class RobotMap {
    public static class Port {
        public static class Network {
            public static SocketAddress LOCAL_SOCKET_ADDRESS = new InetSocketAddress(3375);
            public static SocketAddress LOCALIZATION_ADDRESS = new InetSocketAddress("10.49.04.10", 4321);
        }

        public static class HumanInput {
            public static final int joystick = 0;
            public static final int xboxController = 1;
        }

        public static class CANMotor {
            public static final int RIGHT_DRIVE_A = 2; // TODO: Check chassis motor IDs
            public static final int RIGHT_DRIVE_B = 3;
            public static final int LEFT_DRIVE_A = 4;
            public static final int LEFT_DRIVE_B = 5;

            public static final int INTAKE_AXLE_MOTOR = 7; // TODO: set port for axel intake motor

            public static final int INDEXER_HOLDER_MOTOR = 13; // TODO: set port
            public static final int INDEXER_BELT_MOTOR = 12; // TODO: set port

            public static final int TURRET_MOTOR = 15; // TODO: set port

            public static final int CLIMBER_MOTOR = 9; // TODO: set port

            public static final int SHOOTER_MOTOR = 8; // TODO: set port
        }

        public static class PWM {
        }

        public static class CAN {
        }

        public static class Pneumatics {
            public static final PCMPort INTAKE_EXTENDER_1 = new PCMPort(0, PneumaticsModuleType.CTREPCM, 2, 1); //TODO: set port for drawbridge intake solenoid
            public static final PCMPort INTAKE_EXTENDER_2 = new PCMPort(0, PneumaticsModuleType.CTREPCM, 3, 4); //TODO: set port for drawbridge intake 
            public static final PCMPort SHIFTER = new PCMPort(0, PneumaticsModuleType.CTREPCM, 0, 7); // TODO fix these values maybe?
        }

        public static class Digital {
        }
    }

    public static class Metrics {
        public static class Chassis {
            public static final double DIAMETER_METERS = Units.inchesToMeters(5.0); // TODO: Check values
            public static final double CIRCUMFERENCE_METERS = Metrics.Chassis.DIAMETER_METERS * Math.PI;
            public static final double TICKS_PER_METER = Metrics.Encoders.TalonEncoders.TICKS_PER_REVOLUTION
                    / Metrics.Chassis.CIRCUMFERENCE_METERS;
            public static final double DISTANCE_FRONT_BACK = Units.inchesToMeters(31.0); // TODO: DOUBLE CHECK DISTANCES
            public static final double DISTANCE_SIDE_SIDE = Units.inchesToMeters(28.0); // The robot's a square
            public static final double METERS_PER_TICK = Metrics.Chassis.CIRCUMFERENCE_METERS
                    / Metrics.Encoders.TalonEncoders.TICKS_PER_REVOLUTION;
            public static final double TURN_CORRECTION = 0.0;
        }

        public static class Encoders {
            public static class TalonEncoders {
                public static final double TICKS_PER_REVOLUTION = 2048.0;
                public static final double REVOLUTIONS_PER_TICK = 1 / TICKS_PER_REVOLUTION;
            }
        }
    }

    public static class PID {
        public static class Drive {
        }

        public static class Turn {
        }

        public static class Turret {
            public static final double P = 1e-4; // TODO: TUNE (6e-5)
            public static final double I = 0; // 3E-8
            public static final double D = -5e-6; // (2e-6)
            public static final double F = 0;
            // public static final double tolerance = -1;
            // public static final double dTolerance = -1;

        }

        // public static class Shooter {
        //     public static final double P = 3e-5; // TODO: TUNE (6e-5)
        //     public static final double I = 0; // 3E-8
        //     public static final double D = 0; // (2e-6)
        //     public static final double F = 0;
        //     // public static final double tolerance = -1;
        //     // public static final double dTolerance = -1;

        // }

    }

    public static class Component {
        public static CANTalonEncoder leftWheelTalonEncoder;
        public static CANTalonEncoder rightWheelTalonEncoder;
        public static EncoderPair chassisTalonEncoders;
        public static Motor rightWheelA;
        public static Motor rightWheelB;
        public static Motor leftWheelA;
        public static Motor leftWheelB;
        public static SensorDrive sensorDrive;
        public static TankDrive chassis;
        public static CustomPIDController drivePID;
        public static NavX navx;

        public static CANTalonFX intakeAxleTalon;
        public static Motor intakeAxleMotor;
        public static SolenoidSubsystem intakeExtender1;
        public static SolenoidSubsystem intakeExtender2;
        public static SolenoidSubsystem chassisSolenoid;

        public static CANTalonFX indexerHolderTalon;
        public static CANTalonFX indexerBeltTalon;
        public static Indexer indexer;

        public static Turret turret;
        public static CustomPIDController turretPID;
        public static CANTalonEncoder turretEncoder;
        public static CANTalonFX turretMotor;

        public static CANTalonEncoder shooterEncoder;    
        public static CANTalonFX shooterTalon;
        public static Motor shooterMotor;
        public static CustomPIDController shooterPID;


        // public static SolenoidShifters shifter;
        
        public static RobotUDP robotUDP;
        public static Pose2d initialPose;

        public static CANTalonFX climberTalon;
        public static Motor climberMotor;
        public static Climber climber;
        public static Shooter shooter;
    }

    public static class NetworkTables {
        public static NetworkTableInstance instance;

        public static class Odometry {
            public static NetworkTable table;
            public static NetworkTableEntry pose;
            public static NetworkTableEntry accel;
            public static NetworkTableEntry turretAngle;
        }

        public static class Localization {
            public static NetworkTable table;
            public static NetworkTableEntry goalDistance;
            public static NetworkTableEntry goalRelativeAngle;
        }
    }

    public static class Input {
    }

    public static class HumanInput {
        public static class Driver {
            public static CustomXbox xbox;
        }

        public static class Operator {
            public static CustomJoystick joystick;
        }
    }

    public RobotMap() {
        Component.navx = new NavX(SerialPort.Port.kMXP);

        HumanInput.Driver.xbox = new CustomXbox(Port.HumanInput.xboxController);
		HumanInput.Operator.joystick = new CustomJoystick(Port.HumanInput.joystick);

        Component.intakeAxleTalon = new CANTalonFX(Port.CANMotor.INTAKE_AXLE_MOTOR);
        Component.intakeAxleTalon.configVoltageCompSaturation(11);
        Component.intakeAxleTalon.enableVoltageCompensation(true);

        Component.intakeExtender1 = new SolenoidSubsystem("Intake Extender 1", false, SolenoidState.RETRACT, Port.Pneumatics.INTAKE_EXTENDER_1.buildDoubleSolenoid()); //TODO: check if CANTalonFX or SRX
        Component.intakeAxleMotor = new Motor("Intake Motor", true, Component.intakeAxleTalon);

        Component.indexerHolderTalon = new CANTalonFX(Port.CANMotor.INDEXER_HOLDER_MOTOR);
        Component.indexerHolderTalon.configVoltageCompSaturation(11);
        Component.indexerHolderTalon.enableVoltageCompensation(true);

        Component.indexerBeltTalon = new CANTalonFX(Port.CANMotor.INDEXER_BELT_MOTOR);
        Component.indexerBeltTalon.configVoltageCompSaturation(11);
        Component.indexerBeltTalon.enableVoltageCompensation(true);

        Motor indexerHolderMotor = new Motor("Indexer 1", false, Component.indexerHolderTalon);
        Motor indexerBeltMotor = new Motor("Indexer 2", false, Component.indexerBeltTalon);
        Component.indexer = new Indexer(indexerHolderMotor, indexerBeltMotor);

        Component.climberTalon = new CANTalonFX(Port.CANMotor.CLIMBER_MOTOR);
        Component.climberMotor = new Motor("Climber Motor", false, Component.climberTalon);
        Component.climber = new Climber(Component.climberMotor, Component.climberTalon);


        Component.shooterTalon = new CANTalonFX(Port.CANMotor.SHOOTER_MOTOR);
        Component.shooterMotor = new Motor("ShooterMotor", true, Component.shooterTalon);
        Component.shooterTalon.configVoltageCompSaturation(10);
        Component.shooterTalon.enableVoltageCompensation(true);
        // Component.shooterEncoder = new CANTalonEncoder(Component.shooterTalon);
        // Component.shooterPID = new CustomPIDController(PID.Shooter.P, PID.Shooter.I, PID.Shooter.D, PID.Shooter.F, Component.shooterEncoder);
        // VelocitySensorMotor shooterVSM = new VelocitySensorMotor("Turret", Component.shooterPID, Component.shooterTalon); //TODO: cringe (zach)
        // Component.shooter = new Shooter(shooterVSM, Component.shooterEncoder);

        // Component.shifter = new SolenoidShifters(Port.Pneumatics.SHIFTER.buildDoubleSolenoid());
        
        Component.turretMotor = new CANTalonFX(Port.CANMotor.TURRET_MOTOR);
        Component.turretMotor.setSelectedSensorPosition(0);
        Component.turretEncoder = new CANTalonEncoder(Component.turretMotor);
        Component.turretPID = new CustomPIDController(PID.Turret.P,
                PID.Turret.I, PID.Turret.D, PID.Turret.F,
                Component.turretEncoder);
        Component.turretPID.setAbsoluteTolerance(2048);
        Component.turretPID.setMinimumNominalOutput(0.065); // formerly 0.07
        Component.turretPID.setOutputRange(-0.10, 0.10);
        PositionSensorMotor turretPSM = new PositionSensorMotor("Turret", Component.turretPID, Component.turretMotor);
        Component.turret = new Turret(turretPSM, Component.turretEncoder);


        // UDP things
        try {
            Component.robotUDP = new RobotUDP(Port.Network.LOCAL_SOCKET_ADDRESS, Port.Network.LOCALIZATION_ADDRESS);
        } catch (IOException ex) {
            LogKitten.f("Failed to initialize UDP subsystem");
            LogKitten.ex(ex);
        }

        // Chassis

        /* Drive Train */
        // TalonFX
        CANTalonFX leftWheelATalon = new CANTalonFX(Port.CANMotor.LEFT_DRIVE_A);
        CANTalonFX leftWheelBTalon = new CANTalonFX(Port.CANMotor.LEFT_DRIVE_B);
        CANTalonFX rightWheelATalon = new CANTalonFX(Port.CANMotor.RIGHT_DRIVE_A);
        CANTalonFX rightWheelBTalon = new CANTalonFX(Port.CANMotor.RIGHT_DRIVE_B);

        // Wheels
        Component.rightWheelA = new Motor("rightWheelA", false, rightWheelATalon);
        Component.rightWheelB = new Motor("rightWheelB", false, rightWheelBTalon);
        Component.leftWheelA = new Motor("leftWheelA", true, leftWheelATalon);
        Component.leftWheelB = new Motor("leftWheelB", true, leftWheelBTalon);

        // Wheel Encoders
        Component.leftWheelTalonEncoder = new CANTalonEncoder("leftWheel", leftWheelATalon, true,
                Metrics.Chassis.METERS_PER_TICK);
        Component.rightWheelTalonEncoder = new CANTalonEncoder("rightWheel", rightWheelATalon, true,
                Metrics.Chassis.METERS_PER_TICK);
        Component.initialPose = new Pose2d(); // TODO double x, double y, rotation2d
        Component.sensorDrive = new SensorDrive(Component.chassis, Component.leftWheelTalonEncoder,
                Component.rightWheelTalonEncoder, Component.navx, Component.initialPose);

        Component.chassisTalonEncoders = new EncoderPair(Component.leftWheelTalonEncoder,
                Component.rightWheelTalonEncoder);

        // General Chassis
        Component.chassisSolenoid = new SolenoidSubsystem("Intake Extender 1", false, SolenoidState.EXTEND, Port.Pneumatics.SHIFTER.buildDoubleSolenoid());
        Component.chassis = new TankDrive("2022-Chassis", Component.leftWheelA, Component.leftWheelB,
                Component.rightWheelA, Component.rightWheelB);//, Component.shifter);
        Component.chassis.setDefaultCommand(new ChassisMove(Component.chassis, new NathanGain()));

        // NetworkTables setup
    }
}
