package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
    
public class Tarzan extends SubsystemBase {
    private TalonFX hugMotor;
    private double hugSpeed;
    
    public Tarzan(Config config) {
        var hugMotorPort = config.readIntegerProperty("ports.hug.motor");
        hugMotor = new TalonFX(hugMotorPort);
    
        var limitConfigs = new CurrentLimitsConfigs();
    
        limitConfigs.StatorCurrentLimit = config.readIntegerProperty("tarzan.motor.statorCurrent.limit.amps");
        limitConfigs.StatorCurrentLimitEnable = true;
    
        limitConfigs.SupplyCurrentLimit = config.readIntegerProperty("tarzan.motor.supplyCurrent.limit.amps");
        limitConfigs.SupplyCurrentLimitEnable = true;
    
        hugMotor.getConfigurator().apply(limitConfigs);
    
        hugSpeed = config.readDoubleProperty("tarzan.motor.hug.speed");
        }
    
        public Command hug() {
            return this.runEnd(() -> hugMotor.set(hugSpeed), this::stop);
        }
     
        private void stop() {
            hugMotor.set(0);
        }
    }
    

