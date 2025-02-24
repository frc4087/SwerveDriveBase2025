package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

public class SirLiftsALot extends SubsystemBase {
    private TalonFX climbMotor;
    private Integer climbUpSpeed;
    private Integer climbDownSpeed;

    public SirLiftsALot(Config config) {
        var climbMotorPort = config.readIntegerProperty("ports.climb.motor");
        climbMotor = new TalonFX(climbMotorPort);

        var limitConfigs = new CurrentLimitsConfigs();

        limitConfigs.StatorCurrentLimit = config.readIntegerProperty("sirLiftsALot.motor.statorCurrent.limit.amps");
        limitConfigs.StatorCurrentLimitEnable = true;

        limitConfigs.SupplyCurrentLimit = config.readIntegerProperty("sirLiftsALot.motor.supplyCurrent.limit.amps");
        limitConfigs.SupplyCurrentLimitEnable = true;

        climbMotor.getConfigurator().apply(limitConfigs);

        climbUpSpeed = config.readIntegerProperty("sirLiftsALot.motor.climb.up.speed");
        climbDownSpeed = config.readIntegerProperty("sirLiftsALot.motor.climb.down.speed");
    }

    public Command climberGo() {
        return this.runEnd(() -> climbMotor.set(climbUpSpeed), this::stop);
    }

    public Command climberStop() {
        return this.runEnd(() -> climbMotor.set(climbDownSpeed), this::stop);
    }

    private void stop() {
        climbMotor.set(0);
    }
}
