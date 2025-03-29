package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class SirLiftsALot extends SubsystemBase {
    private TalonFX climbMotor;
    private double climbSpeed;

    public SirLiftsALot(Config config) {
        var climbMotorPort = config.readIntegerProperty("ports.climb.motor");
        climbMotor = new TalonFX(climbMotorPort);

        var limitConfigs = new CurrentLimitsConfigs();

        limitConfigs.StatorCurrentLimit = config.readIntegerProperty("sirLiftsALot.motor.statorCurrent.limit.amps");
        limitConfigs.StatorCurrentLimitEnable = true;

        limitConfigs.SupplyCurrentLimit = config.readIntegerProperty("sirLiftsALot.motor.supplyCurrent.limit.amps");
        limitConfigs.SupplyCurrentLimitEnable = true;

        climbMotor.getConfigurator().apply(limitConfigs);

        climbSpeed = config.readDoubleProperty("sirLiftsALot.motor.climb.speed");

        climbMotor.setNeutralMode(NeutralModeValue.Brake);
        
    }
    
    public Command runClimberForward() {
        return this.runEnd(() -> climbMotor.set(climbSpeed), this::stop);
    }

    public Command runClimberBackward() {
        return this.runEnd(() -> climbMotor.set(-(0.75 * climbSpeed)), this::stop);
    }
 
    private void stop() {
        climbMotor.set(0);
    }
}
