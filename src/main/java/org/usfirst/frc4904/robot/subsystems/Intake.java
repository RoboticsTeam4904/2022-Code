package org.usfirst.frc4904.robot.subsystems;
import org.usfirst.frc4904.standard.subsystems.SolenoidSubsystem;
import org.usfirst.frc4904.standard.subsystems.motor.Motor;

public class Intake {
    public final static double DEFAULT_INTAKE_MOTOR_SPEED = 0.0; //needs value
    // public final static double DEFAULT_DRAWBRIDGE_MOTOR_SPEED = 0.0; //needs value
    public final static double DEFAULT_OFF_SPEED = 0.0;

    public final SolenoidSubsystem drawbridgeSolenoid;
    public final Motor axelMotor;
    
    public Intake(Motor axelMotor, SolenoidSubsystem drawbridgeSolenoid) {
        this.axelMotor = axelMotor;
        this.drawbridgeSolenoid = drawbridgeSolenoid;
    }
}
