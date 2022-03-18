package org.usfirst.frc4904.robot;
import org.usfirst.frc4904.standard.custom.controllers.CustomJoystick;
import org.usfirst.frc4904.standard.custom.controllers.CustomXbox;
import org.usfirst.frc4904.standard.custom.motioncontrollers.CANTalonFX;
import org.usfirst.frc4904.standard.subsystems.motor.Motor;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import org.usfirst.frc4904.robot.commands.shooter.ShooterBrake;
import org.usfirst.frc4904.robot.humaninterface.drivers.NathanGain;
import org.usfirst.frc4904.robot.subsystems.Indexer;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import org.usfirst.frc4904.standard.commands.chassis.ChassisMove;
import org.usfirst.frc4904.standard.custom.PCMPort;
import org.usfirst.frc4904.standard.custom.motioncontrollers.CANTalonSRX;
import org.usfirst.frc4904.standard.custom.motioncontrollers.CustomPIDController;
import org.usfirst.frc4904.standard.subsystems.SolenoidSubsystem.SolenoidState;
import org.usfirst.frc4904.standard.subsystems.SolenoidSubsystem;
import org.usfirst.frc4904.standard.subsystems.chassis.TankDrive;

import org.usfirst.frc4904.standard.custom.sensors.EncoderPair;
import org.usfirst.frc4904.standard.custom.sensors.CANTalonEncoder;
import edu.wpi.first.wpilibj.SerialPort;
import org.usfirst.frc4904.standard.subsystems.chassis.TankDrive;
import org.usfirst.frc4904.standard.subsystems.motor.Motor;
import org.usfirst.frc4904.standard.custom.sensors.NavX;

import org.usfirst.frc4904.standard.subsystems.chassis.SensorDrive;

import org.usfirst.frc4904.standard.subsystems.motor.PositionSensorMotor;
import org.usfirst.frc4904.robot.subsystems.Turret;

public class RobotMap {
    public static class Port {
        public static class HumanInput {
            public static final int joystick = 0;
            public static final int xboxController = 1;
        }

        public static class CANMotor {
            public static final int RIGHT_DRIVE_A = 2; // TODO: Check chassis motor IDs
            public static final int RIGHT_DRIVE_B = 3;
            public static final int LEFT_DRIVE_A = 4;
            public static final int LEFT_DRIVE_B = 5;

            public static final int INTAKE_AXLE_MOTOR = 7; //TODO: set port for axel intake motor

            public static final int INDEXER_HOLDER_MOTOR = 13; // TODO: set port
            public static final int INDEXER_BELT_MOTOR = 12; // TODO: set port

            public static final int TURRET_MOTOR = 15; // TODO: set port

            public static final int SHOOTER_MOTOR = 8; // TODO: set port
        }

        public static class PWM {
        }

        public static class CAN {
        }

        public static class Pneumatics {
            public static final PCMPort INTAKE_EXTENDER_1 = new PCMPort(0, PneumaticsModuleType.CTREPCM, 0, 1); //TODO: set port for drawbridge intake solenoid
            public static final PCMPort INTAKE_EXTENDER_2 = new PCMPort(0, PneumaticsModuleType.CTREPCM, 2, 3); //TODO: set port for drawbridge intake solenoid
        }

        public static class Digital {
        }
    }

    public static class Metrics {
        public static class Chassis {
            public static final double DIAMETER_METERS = Units.inchesToMeters(-1.0); // TODO: Check values 
            public static final double CIRCUMFERENCE_METERS = Metrics.Chassis.DIAMETER_METERS * Math.PI;
            public static final double TICKS_PER_METER = Metrics.Encoders.CANCoders.TICKS_PER_REVOLUTION
                    / Metrics.Chassis.CIRCUMFERENCE_METERS;
            public static final double DISTANCE_FRONT_BACK = Units.inchesToMeters(-1.0); // TODO: DOUBLE CHECK DISTANCES
            public static final double DISTANCE_SIDE_SIDE = Units.inchesToMeters(-1.0); // The robot's a square
            public static final double METERS_PER_TICK = Metrics.Chassis.CIRCUMFERENCE_METERS
                    / Metrics.Encoders.CANCoders.TICKS_PER_REVOLUTION;
            public static final double TURN_CORRECTION = 0.0;
        }

        public static class Encoders {
            public static class TalonEncoders {
                public static final double TICKS_PER_REVOLUTION = 2048.0;
                public static final double REVOLUTIONS_PER_TICK = 1 / TICKS_PER_REVOLUTION;
            }

            public static class CANCoders {
                public static final double TICKS_PER_REVOLUTION = 4096.0;
                public static final double REVOLUTIONS_PER_TICK = 1 / TICKS_PER_REVOLUTION;
            }
        }
    }

    public static class PID {
        public static class Flywheel {
            public static final double P = -1; // TODO: tune PID constants
            public static final double I = -1;
            public static final double D = -1;
            public static final double F = -1;
        }

        public static class Drive {
        }

        public static class Turn {
        }

        public static class Turret {
            public static final double P = -1; // TODO: TUNE
            public static final double I = -1;
            public static final double D = -1;
            public static final double F = -1;
            public static final double tolerance = -1;
            public static final double dTolerance = -1;
        }

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

        public static Motor intakeAxleMotor;
        public static SolenoidSubsystem intakeExtender1;
        public static SolenoidSubsystem intakeExtender2;
        
        public static CANTalonFX indexerHolderTalon;
        public static CANTalonFX indexerBeltTalon;
        public static Indexer indexer;

        public static Turret turret;
        public static CustomPIDController turretPID;
        public static CANTalonEncoder turretEncoder;
        public static CANTalonFX turretMotor;

        public static CANTalonFX shooterTalon;
        public static Motor shooterMotor;
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
        
        Component.intakeExtender1 = new SolenoidSubsystem("Intake Extender 1", false, SolenoidState.RETRACT, Port.Pneumatics.INTAKE_EXTENDER_1.buildDoubleSolenoid()); //TODO: check if CANTalonFX or SRX
        Component.intakeExtender2 = new SolenoidSubsystem("Intake Extender 2", false, SolenoidState.RETRACT, Port.Pneumatics.INTAKE_EXTENDER_2.buildDoubleSolenoid()); //TODO: check if CANTalonFX or SRX
        Component.intakeAxleMotor = new Motor("Intake Motor", true, new CANTalonFX(Port.CANMotor.INTAKE_AXLE_MOTOR)); //TODO: check if CANTalonFX or SRX

        Component.indexerHolderTalon = new CANTalonFX(Port.CANMotor.INDEXER_HOLDER_MOTOR);
        Component.indexerBeltTalon = new CANTalonFX(Port.CANMotor.INDEXER_BELT_MOTOR);
        Motor indexerHolderMotor = new Motor("Indexer 1", false, Component.indexerHolderTalon);
        Motor indexerBeltMotor = new Motor("Indexer 2", false, Component.indexerBeltTalon);
        Component.indexer = new Indexer(indexerHolderMotor, indexerBeltMotor);
        
        Component.turretMotor = new CANTalonFX(Port.CANMotor.TURRET_MOTOR);
        Component.turretEncoder = new CANTalonEncoder(Component.turretMotor, Turret.TICK_MULTIPLIER);
        Component.turretPID = new CustomPIDController(PID.Turret.P,
                PID.Turret.I, PID.Turret.D, PID.Turret.F,
                Component.turretEncoder);
        Component.turret = new Turret(new PositionSensorMotor("Turret", Component.turretPID, Component.turretMotor));
        
        Component.shooterTalon = new CANTalonFX(Port.CANMotor.SHOOTER_MOTOR);
        Component.shooterMotor = new Motor("Shooter", true, Component.shooterTalon);

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
                Metrics.Encoders.TalonEncoders.REVOLUTIONS_PER_TICK);
        Component.rightWheelTalonEncoder = new CANTalonEncoder("rightWheel", rightWheelATalon, true,
                                                               Metrics.Encoders.TalonEncoders.REVOLUTIONS_PER_TICK);      
        Component.sensorDrive = new SensorDrive(Component.chassis, Component.leftWheelTalonEncoder,
        Component.rightWheelTalonEncoder, Component.navx);

        Component.chassisTalonEncoders = new EncoderPair(Component.leftWheelTalonEncoder, Component.rightWheelTalonEncoder);

        // General Chassis
        Component.chassis = new TankDrive("2022-Chassis", Component.leftWheelA, Component.leftWheelB,
                Component.rightWheelA, Component.rightWheelB);
        Component.chassis.setDefaultCommand(new ChassisMove(Component.chassis, new NathanGain()));
    }
}
