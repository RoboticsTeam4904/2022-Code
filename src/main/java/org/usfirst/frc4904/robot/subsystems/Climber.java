package org.usfirst.frc4904.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.usfirst.frc4904.standard.custom.motioncontrollers.CANTalonFX;
import org.usfirst.frc4904.standard.subsystems.motor.Motor;


public class Climber extends SubsystemBase {

  public static final double DEFAULT_CLIMBER_SPEED = 0.5; //TODO: Set value
  public final Motor climberMotor;
  public final CANTalonFX climberTalon;
  public Climber(Motor climberMotor, CANTalonFX climberTalon) {
    this.climberMotor = climberMotor;
    this.climberTalon = climberTalon;
  }
} 
