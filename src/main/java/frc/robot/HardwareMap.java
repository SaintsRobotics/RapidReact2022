package frc.robot;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;


public class HardwareMap {

    public class ShooterHardware{
        public  WPI_TalonFX flywheel;
        public double velocity;
        

        public ShooterHardware(){
            flywheel = new WPI_TalonFX(0);          

        }

    }
    
    public class InputHardware {
        public XboxController driveController;
        public InputHardware() {
            driveController = new XboxController(0);
        }
    }
    ;
    
    public ShooterHardware shooterHardware;
    public InputHardware inputHardware;


    public HardwareMap() {
        shooterHardware = new ShooterHardware();
    }
}