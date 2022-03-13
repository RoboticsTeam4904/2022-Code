package org.usfirst.frc4904.robot;
import org.usfirst.frc4904.standard.custom.controllers.CustomJoystick;
import org.usfirst.frc4904.standard.custom.controllers.CustomXbox;
import org.usfirst.frc4904.standard.custom.motioncontrollers.CANTalonFX;
import org.usfirst.frc4904.standard.subsystems.motor.Motor;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import org.usfirst.frc4904.robot.subsystems.Indexer;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

import org.usfirst.frc4904.standard.custom.PCMPort;
import org.usfirst.frc4904.standard.custom.motioncontrollers.CANTalonSRX;
import org.usfirst.frc4904.standard.custom.motioncontrollers.CustomPIDController;
import org.usfirst.frc4904.standard.subsystems.SolenoidSubsystem.SolenoidState;
import org.usfirst.frc4904.standard.subsystems.SolenoidSubsystem;

import org.usfirst.frc4904.standard.custom.sensors.CANTalonEncoder;
import org.usfirst.frc4904.standard.custom.sensors.CustomCANCoder;
import org.usfirst.frc4904.standard.subsystems.motor.PositionSensorMotor;
import org.usfirst.frc4904.robot.subsystems.Turret;

public class RobotMap {
    public static class Port {
        public static class HumanInput {
            public static final int joystick = 0;
            public static final int xboxController = 1;
        }

        public static class CANMotor {
            public static final int INTAKE_AXLE_MOTOR = -1; //TODO: set port for axel intake motor

            public static final int INDEXER_HOLDER_MOTOR = -1; // TODO: set port
            public static final int INDEXER_BELT_MOTOR = -1; // TODO: set port

            public static final int TURRET_MOTOR = -1; // TODO: set port

            public static final int SHOOTER_MOTOR = -1; // TODO: set port
        }

        public static class PWM {
        }

        public static class CAN {
        }

        public static class Pneumatics {
            public static final PCMPort INTAKE_EXTENDER_1 = new PCMPort(-1, PneumaticsModuleType.CTREPCM, -1, -1); //TODO: set port for drawbridge intake solenoid
            public static final PCMPort INTAKE_EXTENDER_2 = new PCMPort(-1, PneumaticsModuleType.CTREPCM, -1, -1); //TODO: set port for drawbridge intake solenoid
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
        public static Motor intakeAxleMotor;
        public static SolenoidSubsystem intakeExtender1;
        public static SolenoidSubsystem intakeExtender2;
        
        public static Indexer indexer;

        public static Turret turret;
        public static CustomPIDController turretPID;
        public static CANTalonEncoder turretEncoder;
        public static CANTalonFX turretMotor;

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
        HumanInput.Driver.xbox = new CustomXbox(Port.HumanInput.xboxController);
		HumanInput.Operator.joystick = new CustomJoystick(Port.HumanInput.joystick);
        
        Component.intakeExtender1 = new SolenoidSubsystem("Intake Extender 1", false, SolenoidState.RETRACT, Port.Pneumatics.INTAKE_EXTENDER_1.buildDoubleSolenoid()); //TODO: check if CANTalonFX or SRX
        Component.intakeExtender2 = new SolenoidSubsystem("Intake Extender 2", false, SolenoidState.RETRACT, Port.Pneumatics.INTAKE_EXTENDER_2.buildDoubleSolenoid()); //TODO: check if CANTalonFX or SRX
        Component.intakeAxleMotor = new Motor("Intake Motor", false, new CANTalonFX(Port.CANMotor.INTAKE_AXLE_MOTOR)); //TODO: check if CANTalonFX or SRX

        Motor indexerHolderMotor = new Motor("Indexer 1", false, new CANTalonFX(Port.CANMotor.INDEXER_HOLDER_MOTOR));
        Motor indexerBeltMotor = new Motor("Indexer 2", false, new CANTalonFX(Port.CANMotor.INDEXER_BELT_MOTOR));
        Component.indexer = new Indexer(indexerHolderMotor, indexerBeltMotor);
        
        Component.turretMotor = new CANTalonFX(Port.CANMotor.TURRET_MOTOR);
        Component.turretEncoder = new CANTalonEncoder(Component.turretMotor, Turret.TICK_MULTIPLIER);
        Component.turretPID = new CustomPIDController(PID.Turret.P,
                PID.Turret.I, PID.Turret.D, PID.Turret.F,
                Component.turretEncoder);
        Component.turret = new Turret(new PositionSensorMotor("Turret", Component.turretPID, Component.turretMotor));
        
        Component.shooterMotor = new Motor("Shooter", false, new CANTalonFX(Port.CANMotor.SHOOTER_MOTOR));

        /** Classes */
        // Component.intake = new Intake(Component.intakeRollerMotor,
        // Component.liftBeltMotor, Component.funnelMotor,
        // Component.intakeSolenoid);
    }
}
