package org.usfirst.frc4904.robot.subsystems;

import org.usfirst.frc4904.standard.subsystems.motor.Motor;

public class Intake {
    public final static double DEFAULT_AXEL_MOTOR_SPEED = 0.0; //needs value
    public final static double DEFAULT_DRAWBRIDGE_MOTOR_SPEED = 0.0; //needs value
    public final static double DEFAULT_OFF_SPEED = 0.0;

    public Motor drawbridgeMotor;
    public Motor axelMotor;
    
    public Intake(Motor axelMotor, Motor drawbridgeMotor) {
        this.axelMotor = axelMotor;
        this.drawbridgeMotor = drawbridgeMotor;
    }

    public void setAxelIntakeSpeed(double speed) {
        axelMotor.set(speed);
    }

    public void setDrawBridgeSpeed(double speed) {
        drawbridgeMotor.set(speed);
    }

    public void setSpeed(double axelIntakeSpeed, double drawbridgeIntakeSpeed) {
        setDrawBridgeSpeed(drawbridgeIntakeSpeed);
        setAxelIntakeSpeed(axelIntakeSpeed);
    }

    public void setSpeed() {
        setSpeed(DEFAULT_AXEL_MOTOR_SPEED, DEFAULT_DRAWBRIDGE_MOTOR_SPEED);
    }
    public void stop() {
        setSpeed(DEFAULT_OFF_SPEED, DEFAULT_OFF_SPEED); //maybe retract drawbridge?
    }
    // public void retractDrawBridge(double speed) {
    //     setDrawBridgeSpeed(-speed); //needs value
    // }
    }
