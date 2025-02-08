package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

public class SirLiftsALot extends SubsystemBase {
    private TalonFX climbMotor;
    private Integer climbSpeed;

    public SirLiftsALot(Config config) {
        var climbMotorPort = config.readIntegerProperty("ports.climb.motor");
        climbMotor = new TalonFX(climbMotorPort);

        var limitConfigs = new CurrentLimitsConfigs();

        limitConfigs.StatorCurrentLimit = config.readIntegerProperty("sirLiftsALot.motor.statorCurrent.limit.amps");
        limitConfigs.StatorCurrentLimitEnable = true;

        limitConfigs.SupplyCurrentLimit = config.readIntegerProperty("sirLiftsALot.motor.supplyCurrent.limit.amps");
        limitConfigs.SupplyCurrentLimitEnable = true;

        climbMotor.getConfigurator().apply(limitConfigs);

        climbSpeed = config.readIntegerProperty("sirLiftsALot.motor.climb.speed");
    }

    public Command runClimber() {
        return this.runEnd(() -> climbMotor.set(climbSpeed), this::stop);
    }

    private void stop() {
        climbMotor.set(0);
    }
}
