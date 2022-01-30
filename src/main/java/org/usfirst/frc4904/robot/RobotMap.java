package org.usfirst.frc4904.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import org.usfirst.frc4904.robot.humaninterface.drivers.NathanGain;
import org.usfirst.frc4904.standard.commands.chassis.ChassisMove;
import org.usfirst.frc4904.standard.custom.CustomPIDSourceType;
import org.usfirst.frc4904.standard.custom.controllers.CustomJoystick;
import org.usfirst.frc4904.standard.custom.controllers.CustomXbox;
import org.usfirst.frc4904.standard.custom.motioncontrollers.CANTalonFX;
import org.usfirst.frc4904.standard.custom.motioncontrollers.CustomPIDController;
import org.usfirst.frc4904.standard.custom.sensors.CANTalonEncoder;
import org.usfirst.frc4904.standard.custom.sensors.CustomCANCoder;
import org.usfirst.frc4904.standard.custom.sensors.EncoderPair;
import org.usfirst.frc4904.standard.subsystems.chassis.TankDrive;
import org.usfirst.frc4904.standard.subsystems.motor.Motor;

public class RobotMap {
    public static class Port {
        public static class HumanInput {
            public static final int joystick = 0;
			public static final int xboxController = 1;
        }

        public static class CANMotor {
            public static int RIGHT_DRIVE_A = -1; // TODO: Check chassis motor IDs
            public static int RIGHT_DRIVE_B = -1;
            public static int LEFT_DRIVE_A = -1;
            public static int LEFT_DRIVE_B = -1;
        }

        public static class PWM {
        }

        public static class CAN {
            public static final int LEFT_WHEEL_ENCODER = -1; // TODO: Check CAN IDs
            public static final int RIGHT_WHEEL_ENCODER = -1;
        }

        public static class Pneumatics {
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
        public static class Drive {
        }

        public static class Turn {
        }

    }

    public static class Component {
        public static CANTalonEncoder leftWheelTalonEncoder;
        public static CANTalonEncoder rightWheelTalonEncoder;
        public static CustomCANCoder leftWheelCANCoder;
        public static CustomCANCoder rightWheelCANCoder;
        public static EncoderPair chassisTalonEncoders;
        public static EncoderPair chassisCANCoders;
        public static Motor rightWheelA;
        public static Motor rightWheelB;
        public static Motor leftWheelA;
        public static Motor leftWheelB;
        public static TankDrive chassis;
        public static CustomPIDController drivePID;
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
        HumanInput.Driver.xbox = new CustomXbox(Port.HumanInput.xboxController);
		HumanInput.Operator.joystick = new CustomJoystick(Port.HumanInput.joystick);

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
        Component.leftWheelCANCoder = new CustomCANCoder(Port.CAN.LEFT_WHEEL_ENCODER,
                Metrics.Chassis.METERS_PER_TICK);
        Component.rightWheelCANCoder = new CustomCANCoder(Port.CAN.RIGHT_WHEEL_ENCODER,
                Metrics.Chassis.METERS_PER_TICK);

        Component.leftWheelTalonEncoder = new CANTalonEncoder("leftWheel", leftWheelATalon, true,
                Metrics.Encoders.TalonEncoders.REVOLUTIONS_PER_TICK, CustomPIDSourceType.kDisplacement,
                FeedbackDevice.IntegratedSensor);
        Component.rightWheelTalonEncoder = new CANTalonEncoder("rightWheel", rightWheelATalon, true,
                Metrics.Encoders.TalonEncoders.REVOLUTIONS_PER_TICK, CustomPIDSourceType.kDisplacement,
                FeedbackDevice.IntegratedSensor);

        Component.chassisTalonEncoders = new EncoderPair(Component.leftWheelTalonEncoder, Component.rightWheelTalonEncoder);
        Component.chassisCANCoders = new EncoderPair(Component.leftWheelCANCoder, Component.rightWheelCANCoder);

        // General Chassis
        Component.chassis = new TankDrive("2022-Chassis", Component.leftWheelA, Component.leftWheelB,
                Component.rightWheelA, Component.rightWheelB);
        Component.chassis.setDefaultCommand(new ChassisMove(Component.chassis, new NathanGain()));
    }
}