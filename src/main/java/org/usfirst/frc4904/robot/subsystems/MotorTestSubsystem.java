import edu.wpi.first.wpilibj.motorcontrol.MotorController;

//motor code to turn on and off
//import org.usfirst.frc4904.standard.subsystems.motor;


public class MotorTestSubsystem extends MotorController {
    Motor motor1 = new motor();
    for (MotorController motor : motors) {
        if (motor instanceof IMotorController)
            ((IMotorController) motor).enableVoltageCompensation(true);
        motor.set(5); // Start all motors with 5 speed

        @Override
        public void stopMotor() {
            for (MotorController motor : motors) {
                motor.stopMotor();
            }
        }
    
}