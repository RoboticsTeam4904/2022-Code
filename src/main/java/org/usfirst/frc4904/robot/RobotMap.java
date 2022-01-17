package org.usfirst.frc4904.robot;

import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc4904.standard.custom.controllers.CustomJoystick;
import org.usfirst.frc4904.standard.custom.motioncontrollers.CANTalonFX;
import org.usfirst.frc4904.standard.custom.motioncontrollers.CANTalonSRX;
import org.usfirst.frc4904.standard.custom.motioncontrollers.CustomPIDController;
import org.usfirst.frc4904.standard.custom.controllers.CustomXbox;
import org.usfirst.frc4904.standard.custom.sensors.CANTalonEncoder;
import org.usfirst.frc4904.robot.subsystems.Flywheel;
import org.usfirst.frc4904.standard.custom.CustomPIDSourceType;

public class RobotMap {
    public static class Port {
        public static class HumanInput {
            public static final int joystick = 0;
			public static final int xboxController = 1;
        }

        public static class CANMotor {
            public static final int FLYWHEEL_MOTOR_A = -1; // TODO: set port
            public static final int FLYWHEEL_MOTOR_B = -1;
        }

        public static class PWM {
        }

        public static class CAN {
        }

        public static class Pneumatics {
        }

        public static class Digital {
        }
    }

    public static class Metrics {
        public static class Chassis {
            public static final double TICKS_PER_REVOLUTION = -1; // TODO: CHANGE CONSTS
            public static final double DIAMETER_INCHES = -1;
            public static final double CIRCUMFERENCE_INCHES = Metrics.Chassis.DIAMETER_INCHES * Math.PI;
            public static final double TICKS_PER_INCH = Metrics.Chassis.TICKS_PER_REVOLUTION
                    / Metrics.Chassis.CIRCUMFERENCE_INCHES;
            public static final double DISTANCE_FRONT_BACK = -1;
            public static final double DISTANCE_SIDE_SIDE = -1;
            public static final double INCHES_PER_TICK = Metrics.Chassis.CIRCUMFERENCE_INCHES
                    / Metrics.Chassis.TICKS_PER_REVOLUTION;
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

    }

    public static class Component {
        public static Flywheel flywheel;
        public static CANTalonEncoder flywheelEncoderA;
        public static CANTalonEncoder flywheelEncoderB;
        public static Flywheel shooter;
        public static CustomPIDController flywheelPID;
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

        CANTalonFX flywheelATalon = new CANTalonFX(Port.CANMotor.FLYWHEEL_MOTOR_A);
        // flywheelATalon.setInverted(false);
        CANTalonFX flywheelBTalon = new CANTalonFX(Port.CANMotor.FLYWHEEL_MOTOR_B);

        Component.flywheelEncoderB = new CANTalonEncoder(flywheelBTalon, false,
            Metrics.Encoders.TalonEncoders.REVOLUTIONS_PER_TICK);
        Component.flywheelEncoderB.setCustomPIDSourceType(CustomPIDSourceType.kRate);
        /** Motion Controllers */
        Component.flywheelPID = new CustomPIDController(PID.Flywheel.P, PID.Flywheel.I, PID.Flywheel.D, PID.Flywheel.F,
                Component.flywheelEncoderB);

        Component.shooter = new Flywheel("Shooter", Component.flywheelPID, flywheelATalon, flywheelBTalon);
        /** Classes */
        // Component.intake = new Intake(Component.intakeRollerMotor,
        // Component.liftBeltMotor, Component.funnelMotor,
        // Component.intakeSolenoid);
    }
}
